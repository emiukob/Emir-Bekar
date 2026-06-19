# -*- coding: utf-8 -*-
"""
leader_nav.py — Lider & takipci swarm robot navigasyonu (tek dosya).

Calistirma:
    python leader_nav.py
    python leader_nav.py --camera-index 0 --port COM5
    python leader_nav.py --flat-log        # tek nav_log.txt'ye yaz (uzerine)

GUVENLIK KATMANLARI:
  1) Kare watchdog: son YENI kareden FRAME_STALE_TIMEOUT gecerse tum robotlara
     aktif 0 PWM basilir (Wi-Fi koptu / stream dondu -> robotlar donmus
     goruntuyle surmeye DEVAM ETMEZ) + ekranda kirmizi banner.
  2) SerialWriter deadman: ana dongu donarsa TX thread'i CMD_DEADMAN sn sonra
     kendiliginden 0 basar.
  3) write_timeout: gateway takilsa bile kontrol dongusu kilitlenmez.
  4) Gateway + robot firmware tarafinda komut-timeout failsafe mevcut
     (Typce_C_Gateway.ino / Robot_ESP32_RL*.ino — PC capraz cokse bile dururlar).

FEATURE FLAG'LER (CONFIG bolumu): USE_KALMAN, ADAPTIVE_LOOKAHEAD,
LOG_TIMESTAMPED — donanimda tuhaflik gorursen tek tek kapatip izole et.
"""
import os

# FFmpeg/MJPEG ayarlari — cv2 import'undan ONCE olmali:
#  - LOGLEVEL=-8 (quiet): wifi akisindaki "overread" spam'ini sustur
#  - nobuffer + low_delay: ag kamerasinda gecikmeyi azalt
os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "fflags;nobuffer|flags;low_delay|reorder_queue_size;0",
)

import ctypes
import heapq
import json
import math
import threading
import time
from collections import deque
from statistics import median

import cv2
import numpy as np
import serial
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from pupil_apriltags import Detector

# OpenCV ic loglarini sustur (API surume gore degisiyor; ikisini de dene)
try:
    cv2.setLogLevel(0)
except AttributeError:
    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
    except Exception:
        pass


# ===================== KONFIGURASYON =====================

# ====================== KAMERA KAYNAGI ======================
# IP Webcam (Android uygulamasi): CAMERA_URL'yi doldur, CAMERA_INDEX = None birak.
# USB / yerel webcam icin: CAMERA_URL = "", CAMERA_INDEX = 0/1/2 ...
# (Komut satirindan da verilebilir: --camera-url / --camera-index / --port)
CAMERA_URL   = "http://192.168.1.84:8080/video"   # IP Webcam app (Android)
CAMERA_INDEX = None   # IP kamera modunda kullanilmaz
CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 1280, 720, 30      # IP Webcam'in destekledigi max cozunurluk
GATEWAY_PORT = "COM7"
BAUD_RATE    = 115200
LEADER_ID    = 0
APRILTAG_FAMILIES = "tag36h11"

# --- Kamera kalibrasyonu (lens bozulmasi) ---
# calibrate_KAMERA bolumu ile uretilir; dosya yoksa undistort atlanir (graceful fallback).
# Kalibrasyon 1280x720'de alindi (dogrulandi). Farkli cozunurlukte yeniden alinirsa
# npz icine "image_size" anahtari eklenirse K otomatik olceklenir (KAMERA bolumu).
CALIB_FILE = "camera_calib.npz"

# --- Piksel<->cm olcek ---
# AprilTag'in fiziksel kenar uzunlugu (cm). Tag boyutunuza gore guncelleyin.
# PX_PER_CM her kare gorunen TUM tag'lerin medyanindan canli olculur.
TAG_SIZE_CM = 16.0

# 3/3 park (gorev tamam) sonrasi otomatik kapanmadan once ekranda kalma suresi (sn).
# Son durumu + RAPOR karnesini gormen icin; beklemeden cikmak istersen Q.
AUTO_EXIT_GRACE = 4.0

# ====================== GUVENLIK / FAILSAFE ======================
# Kamera watchdog: son YENI kareden bu sure gecerse tum robotlara 0 PWM basilir.
# (Wi-Fi koptu / stream dondu senaryosu — Nav2 collision monitor "source_timeout" mantigi)
FRAME_STALE_TIMEOUT = 0.5     # sn

# Seri yazma timeout'u: gateway takilirsa write() ana donguyu kilitlemesin.
SERIAL_WRITE_TIMEOUT = 0.05   # sn

# PC-ici deadman: ana dongu (detector vs.) donar da set_pwms cagrilamazsa,
# SerialWriter thread'i bu sureden sonra kendiliginden 0 basar.
CMD_DEADMAN = 0.5             # sn

CMD_PERIOD  = 0.06            # robot basina komut gonderim periyodu (sn) — orijinalle ayni

# !!! ONEMLI: PC tarafi watchdog'lar PC CALISIRKEN korur. PC capraz cokerse /
# USB cekilirse son care ROBOT FIRMWARE'inde komut timeout'udur (~400 ms).
# Robot_ESP32_RL*.ino firmware'inde 500 ms failsafe MEVCUT; gateway de 500 ms

# ====================== FEATURE FLAG'LER ======================
# Donanimda bir sey ters giderse tek tek kapatip izole edebilmen icin:
USE_KALMAN          = True   # False -> eski EMA pozisyon filtresi davranisi (TAKIP bolumu)
ADAPTIVE_LOOKAHEAD  = True   # False -> sabit FOLLOWER_LOOKAHEAD kullanilir
LOG_TIMESTAMPED     = True   # False -> eski gibi tek "nav_log.txt" dosyasina yazar (uzerine)

# ====================== HAREKET / PWM ======================
MIN_PWM = 19          # lider tabani (20:26: 81 px/s -> 19; hedef ~65).
# Takipciler icin DAHA DUSUK taban -> gercekten daha yavas hareket ederler (lider 28'de,
# takipci 22'de). Robotlar PWM tabaninda kosuyordu; tek gercek yavaslatma kaldraci bu.
# !!! STALL RISKI: cok dusuk olursa takipciler hic hareket etmez. Hareket etmiyorlarsa ARTIR
# (24->26->28). Hala hizliysa biraz daha DUSUR (22->20). Gercek motor stall esigine gore ayarla.
FOLLOWER_MIN_PWM = 24    # taban dusuk -> fuzzy bandina yer kalir (hiz esitleme trimlerle)
# Robot-bazli PWM duzeltmesi (taban PWM'e eklenir).
# R3'un yavasligi artik FIRMWARE'de cozuluyor (Robot_ESP32_RL_3.ino HIZ_CARPANI=0.65,
# 2026-06-11 flash). !!! R3 FLASHLANMADIYSA buraya geri 4 yaz (yoksa yine yavas kalir).
# Olculen hizlara gore (20:04): R3=85(guclu, firmware 0.65), R1=50, R2=33(en zayif) px/s.
# Zayif motorlara ek duty: hedef ucunun de ~70-85 bandinda esitlenmesi.
# 20:26 olcumu: R1=13 (STALL! -2 fazlaydi), R2=101, R3=68(hedefte), lider=81.
# R1 stall ustune, R2 asagi; R3 dokunma. NOT: motorlar kosudan kosuya +-%20 oynuyor
# (batarya) — bu band icinde kalmak normal, mukemmel sabitleme donanimsal olarak yok.
# R3 trimi -10 -> 0 (15:20 logu): -10 ile taban 14 PWM statik surtunmeyi
# kiramiyordu (12.8 sn stall). SONRA 16:11 logu (taze batarya): tabanda bile
# R3 125 / R2 92 / R1 64 px/s — esitsiz ve hizli. HIZ ESITLEME taban uzerinden:
# guclu motorlara negatif trim (R3 taban 20, R2 taban 21, R1 taban 25).
# Batarya cokup stall donerse trimleri 0'a geri al (kontrol: kalkista log).
ROBOT_PWM_TRIM = {1: 1, 2: -3, 3: -4}
MAX_PWM = 50
# Ileri hiz "soyut birim" -> PWM kazanci. BUYUTULDU (1.28->2.5): fuzzy'nin
# 0..0.89 cikti bandi tabanin ustunde GERCEK hiz farki yaratsin diye
# (fast~+3 PWM, slow~+1.4, stop=0 -> motor duty'de gorunur kademeler).
PWM_SCALE = 2.5

ROBOT_CALIBRATION = {
    0: {"angle_offset": 0.0,   "left_invert": False, "right_invert": False},
    1: {"angle_offset": 0.0,   "left_invert": False, "right_invert": False},
    2: {"angle_offset": 0.0,   "left_invert": False, "right_invert": False},
    3: {"angle_offset": 180.0, "left_invert": False, "right_invert": False},
}

ROBOT_IDS    = [0, 1, 2, 3]
FOLLOWER_IDS = [1, 2, 3]

FWD_MAX = 4
FWD_CATCHUP = 6        # takipci yakalama (MAX) hizi (8 cok atilgandi). Lider etkilenmez.
TURN_MAX = 1.5
TURN_CATCHUP = 1.8
LEADER_SPEED_SCALE   = 0.42  # liderin hizi (biraz daha yavas -> daha kontrollu, takipciler rahat yetisir)
# 0.50 -> 0.40 (16:11 logu): taze bataryayla takipciler 92-164px/s'e firladi.
# 0.40 -> 0.33 (kullanici: "hala hizli, acelemiz yok"): yavas = gecikme altinda
# az asma, az zikzak. Alt sinir: taban PWM'e dayaninca daha fazla dusmez —
# o noktada kaldiraci trim/firmware'dir, bu sayiyi daha kucultme.
FOLLOWER_SPEED_SCALE = 0.33  # takipci cruise — fuzzy acc_scale (0..0.89) bunun uzerine carpar

# --- Yumusak hizlanma (slew-rate / ivme limiti) ---
# 2 MOTORLU DIFERANSIYEL SURUS: sol/sag tekerin ortak (ileri/oteleme) bileseni "fwd"
# ani sicrarsa teker patinar / arac zibirdar. Fuzzy ACC yumusak bir acc_scale uretiyor
# ama PWM tabani (MIN_PWM) yuzunden bu yumusaklik tekerde ~3 PWM adimina sikisiyordu;
# komutu ZAMANA yayarak (ivme limiti) gercek yumusak hiz gecisi saglanir.
# SADECE HIZLANMA sinirlanir; YAVASLAMA/FREN SERBEST birakilir -> ACC freni, carpisma
# yavaslatmasi ve acil dur GEC KALMAZ (guvenlik). DONUS (turn) hic sinirlanmaz -> direksiyon cevik.
SMOOTH_ACCEL    = True   # False -> ani komut (eski davranis); donanimda tuhaflik olursa kapat
FWD_ACCEL_LIMIT = 14.0   # fwd biriminin sn'de artabilecegi max miktar (~cruise 2.8'e 0.2sn'de ulasir)

# ====================== TESPIT / TAKIP ======================
MIN_DECISION_MARGIN = 35.0  # bu degerin altindaki AprilTag tespiti reddedilir
MAX_JUMP_PX = 120.0         # bir okumada bu kadar px'den fazla ziplama: sahte tespit (KF innovation gate)
LOST_TIMEOUT = 0.30         # tag bu sure (sn) kaybolursa hiz vektoruyle konum tahmin et, sonra dur
MAX_PREDICT_PX = 90.0       # tahminle en fazla bu kadar px ileri git (kacisi onler)
TRACK_REINIT_GAP = 0.5      # bu sureden uzun kayiptan sonra filtre HAM olcumle sifirdan kurulur
                            # (eski bug: EMA bayat degerle harmanlayip 5-8 kare yanlis konum veriyordu)

# Kalman filtre gurultu parametreleri (USE_KALMAN=True iken):
KF_SIGMA_ACC  = 180.0   # px/s^2 — surec gurultusu. Buyut -> daha cevik/titrek, kucult -> daha yumusak/gec.
KF_SIGMA_MEAS = 2.0     # px — AprilTag merkez olcum gurultusu (alt-piksel hassas, 720p'de ~1-3 px)

EMA_ALPHA = 0.35        # aci filtresi (sin/cos EMA) ve USE_KALMAN=False fallback icin

# ====================== NAVIGASYON ======================
# 35 -> 44 (16:19 logu — "ucu ucuna varirken dur-kalk loop"): hedefe 35-50px
# bandinda kamera titremesiyle aci sapip pivot/dur dongusu oluyordu; 44'te
# varis biraz erken kilitlenir, hover-loop bandindan once "vardi" der.
ARRIVAL_RADIUS = 50          # Lider bu mesafede hedefe "varmis" sayilir — donmeyi onler (44->50: 18:29 testinde 72px'de asili kalip pivot loop'a girdi, varisi biraz erken latch'le)
FOLLOWER_LOOKAHEAD = 45      # pure-pursuit havuc mesafesi (ADAPTIVE_LOOKAHEAD=False iken sabit)
# Adaptif lookahead (Nav2 RPP / MIT APP yaklasimi): LA = clamp(t * hiz, min, max)
# UZATILDI: kisa lookahead burnunun dibine baktirip titretiyordu (kamera gecikmesiyle kotulesir).
LOOKAHEAD_TIME   = 0.60      # sn — hizin daha ilerisine bak (yumusak takip)
LOOKAHEAD_MIN_PX = 55.0      # dusuk hizda bile rotayi yumusak izle (titreme yok)
LOOKAHEAD_MAX_PX = 80.0      # yuksek hizda yumusak takip (salinmaz)

PARK_ARRIVAL_RADIUS = 38     # Takipci park slotuna bu mesafede gelince kilitlenir (KUCUK = slota tam otur)
PARK_ENTER_DIST = 240        # Takipci finise (liderin durdugu yere) bu kadar yaklasinca PARK pozisyonuna gecer (6 hucre)
PARK_SPACING = 175.0         # Lider->slot mesafesi (PARK_OFFSETS birimleri bununla carpilir)
PARK_FORCE_AFTER = 12.0      # R1'in arka-slot serbest birakma fallback'i bundan olculur (1.5x)
# SIGORTA artik SURE degil ILERLEME bazli: robot slotuna dogru YOL ALDIGI surece kesilmez
# (eski 12sn sabit sure, kisa parkurda uzaktan gelen robotu YOLDA kilitliyordu — 18:27 testi).
# Sadece GERCEKTEN takilirsa (PARK_STUCK_SEC boyunca PARK_STUCK_MIN_PX'ten az yol) kilitlenir.
PARK_STUCK_SEC    = 9.0      # bu sure boyunca ilerleme yoksa "takildi" say (sabirli)
PARK_STUCK_MIN_PX = 20.0     # "ilerleme" sayilacak minimum yol (px)
SLOW_ZONE = 120              # Daha erken yavasla
# 8 -> 11 (15:02 logu — lider/takipci jitter 60-77 der/adim, asiri sag-sol):
# kucuk aci hatalarinda hic donme -> daha duz seyir. Hem lider hem takipci.
HEADING_DEADZONE = 13.0      # bu acinin altinda DUZ git (kamera gecikmesi/gurultusu motoru yormasin) | geri-al: 11.0
# 16:11 logu (taze batarya, zikzak %25-43, pivot %42-61): sonumleme YETMEDI —
# guclu motorla ayni PWM daha sert dondurur, ayni KD az kalir. Ikisi de artirildi.
LEADER_TURN_KD = 0.28        # 0.24->0.28 (15:02 logu): kamera gecikmeli overshoot/slalomu daha cok sonumle
LEADER_TURN_KP = 0.04        # Lider orantisal donus kazanci dusuk -> genis bant, DUZ gider, sapmaz
FOLLOWER_TURN_KD = 0.22      # Takipci donus sonumlemesi — yalpalama/overshoot'u keser
FOLLOWER_TURN_KP = 0.07      # Takipci orantisal donus kazanci — liderle AYNI (az agresif, az overshoot)
# Takipci pivot esigi 65 -> 80 (liderdeki fix'in aynisi; lider 12->1.9 jitter).
# Sim guvenlik A/B (65 vs 80, 2 parkur): park 3/3 ve min mesafe >=67px (temas yok)
# -> guvenli. Yumuszaklik gercekte cikar (liderdeki gibi; sim gecikmeyi modellemez).
# Sorun gorulurse 65'e geri al (tek sabit).
FOLLOWER_PIVOT_DEG = 80.0

ACTIVATION_MAX_WAIT = 2.5    # mesafe kosulu baskin; bu sadece onceki takilirsa son care fallback
# Sira: Lider -> R3 -> R1 -> R2 (R3 EN YAVAS oldugu icin ILK cikar — avans alir).
PREV_FOLLOWER = {1: 3, 2: 1}

# Carpisma onleme: bir takipci, kendisinden YUKSEK oncelikli ve hareket yonunde (on koni)
# bu mesafeden yakin bir robot varsa kademeli yavaslar. Dusuk oncelikli yol verir -> deadlock olmaz.
COLLISION_STOP_DIST = 120.0  # Dur-freni artik SON CARE (once ROBOT_REPULSE direksiyonla kacinir)
# Convoy front->back sirasiyla AYNI olmali: lider(1x) > R1(1x) > R2(2x) > R3(3x).
# Boylece her takipci ONUNDEKI robota yol verir (arkadan carpmaz). YANLIS sira -> R3 R2'ye carpardi.
COLLISION_PRIORITY = [0, 1, 2, 3]   # yuksek -> dusuk (Lider, R1, R2, R3)
COLLISION_CONE_DOT = 0.15    # cos(~81 der): cok genis YAN acidaki robotu "onumde" sanip bosuna durmasin
# ANTI-RAM (karsilikli yakin-alan freni): oncelik mantigi convoy DUZ giderken dogru
# (herkes ONUNDEKINE yol verir). Ama EGRI/geri-donuslu yolda (orn. koridor) arc-uzunluk
# formasyon slotlari mekansal olarak yer degistirebilir -> ust-oncelikli robot (R2)
# alt-onceklinin (R3) USTUNE gidebilir; eski kod buna fren yapmiyordu (sadece ust-onceliye bakar).
# Bu yuzden: ONCELIK FARKETMEKSIZIN, kendi gidis yonumdeki ('ahead') ve bu mesafeden yakin
# HER robota fren. Yon konisi -> yalnizca USTUNE GIDEN durur, digeri durmaz (deadlock olmaz).
COLLISION_MUTUAL_DIST = 110.0  # SON CARE dur bandi (once kacinma direksiyonu calisir; cut ~87px)
COLLISION_MUTUAL_DOT  = 0.30   # cos(~72 derece): "net onumde" sayilma esigi (yan-yana gecisi engellemez)
# HAYALET ROBOT: tag'i kaybolan robota 0 PWM basilir -> OLDUGU YERDE durur. Son bilinen
# konumu carpisma icin bu kadar sure GECERLI sayilir. (18:02 testi: R1 tag kaybetti,
# eski 1.0s pencere doldu, R2 gorunmez R1'in 40px dibine kadar itti = fiziksel temas!)
COLLISION_GHOST_SEC = 30.0
# Temas cemberi payi: contact = 2*ROBOT_RADIUS + bu pay (sabit 22 idi; A/B
# deneyleri icin sabite cikarildi — varsayilan davranis AYNI).
COLLISION_CONTACT_PAD = 22.0
# AKTIF KACINMA (kullanici istegi): carpisma yaklasirken DURMAK yerine en yakin
# robottan ZIT yone donup uzaklas. Duvar APF'sinin robotlar-arasi esdegeri;
# itme vektoru hedef yonune karisir -> robot kavis cizip dolanir. Dur-freni
# (collision_scale) sadece bu kacinma yetmezse SON CARE olarak devreye girer.
ROBOT_REPULSE_DIST = 140.0   # bu mesafeden itibaren diger robottan kacin
ROBOT_REPULSE_GAIN = 1.8     # itme siddeti (2.5 salinim yaratiyordu; yumusak kacis)

# ====================== DUVAR SISTEMI ======================
ROBOT_RADIUS_PX  = 26    # robotun yarisi (~16cm/2 + pay)
WALL_SAFETY_PX   = 20    # ekstra pay: lider salinimi + takip titremesi (ASLA degmesin)
WALL_WP_RADIUS   = 14    # (rota planlama ic hesabi icin)
# 28 -> 40 GERI ALINDI (16:46 logu, kullanici: "basit/duz parkurda waypoint
# lidere sorun cikariyor, eski daha iyiydi"): 28'de lider rotanin ufak orgusune
# bile SIKI sadik kaliyor -> basit parkurda sag-sol gidiyor. 40'ta KOSE KESEREK
# daha duz fiili yol cizer (eski/bilinen-iyi). Sim S-viraj sapmasi biraz artabilir
# ama kullanici basit-parkur duzlugunu oncelikliyor.
LEADER_WP_RADIUS = 40    # lider ara-waypoint'e bu mesafede "gecti" sayilir (durum gostergesi)
# TAKIPCI CROSS-TRACK: rotaya yanal hata buyukse lookahead'i kisalt -> cizgiye HIZLI
# oturur (dagiNik baslangic/sapma sonrasi). Sim'de vardi, leader_nav'da eksikti -> takipci
# cizgiye gec oturup T-kapida line olamiyordu. crosstrack_lookahead_factor zaten tanimli.
XTRACK_REF = 90.0        # bu yanal hatada (px) lookahead tam kisilir (xte_norm=1.0)
XTRACK_LA_MIN = 14.0     # lookahead alt tabani (cok kisa -> carrot jitter)
_XTE_EMA = {}            # rid -> yumusatilmis yanal hata (ani lookahead sicramasi yok)
_TRI_COH_PHASE = {}      # rid -> UCGEN cohesion DUTY faz biriktirici (yavaslat, DURDURMA)
LEADER_LOOKAHEAD = 65    # PURE-PURSUIT: lider rotada bu kadar ILERIDEKI carrot'a kilitlenir | geri-al: 50
# (anlik waypoint yerine) -> bearing kararli, kavisli yumusak donus, pivot-kilidi yok.
# Kucuk=sadik ama hassas; buyuk=yumusak ama kose keser. ~50 dengeli (takipci 48 ile uyumlu).
# Yol segmentleri duvardan en az (yaricap+pay) uzak; waypoint'ler kose kesmeyi de hesaba katar.
WALL_PATH_MARGIN = ROBOT_RADIUS_PX + WALL_SAFETY_PX           # = 46  (merkez bu kadar uzak -> kenar degmez)
WALL_CLEARANCE   = WALL_PATH_MARGIN + WALL_WP_RADIUS          # = 60  (waypoint kose offset'i)
# Lider duvar itmesi (APF): takipciden DAHA ERKEN + DAHA YUMUSAK. Varsayilan (64px, 3.2)
# liderde GEC ve ANI donduruyordu; erken/hafif itme engele gelmeden kademeli kavis cizdirir,
# iki duvar arasinda da lideri bosluk ORTASINA merkezler.
# 155 -> 110 (15:02 logu): koridorda lider hem ust hem alt duvari 155px'den birden
# hissedip net itme yon degistiriyordu -> sol-sag salinim (y'de 100px). 110 ile
# sadece gercekten yakinken tepki -> koridorda DUZ gider (sim A/B: y-salinim 35->26).
LEADER_WALL_INFLUENCE = 110.0   # px — itme yaklasinca baslar (koridorda salinim azalir)
LEADER_WALL_GAIN      = 1.0     # itme daha HAFIF (ani sapma yerine kademeli yay) | geri-al: 1.5
# ITME OLU-BANDI (16:19 — T-kapi/dar gecit zigzag fix'i): dar gecitte lider tam
# ORTADAYKEN iki duvar da zayifca iter, NET kucuk ama isaret degistirir -> kamera
# gecikmesiyle sol-sag sinir dongusu. Net itme bu esigin altindaysa YOK SAY (lider
# planli guvenli merkez cizgiyi izler); sadece bir duvara GERCEKTEN yaklasinca
# (net buyuk) duzelt. Yumusak esik (cikarma). Asiri buyutme: gec tepki/duvara surtme.
LEADER_WALL_DEADBAND  = 0.55    # bu net itme buyuklugu altinda APF devre disi.
# 0.35->0.55 GERI ALINDI (2026-06-14): 0.35 koridor/gate'te merkez-flip zigzag'ini
# yeterince bastiramiyordu -> lider duvar yaninda KESKIN/ani dondu (son loglar: ani
# donuslerin 5/5'i duvara <70px). 0.55 liderin "mukemmel/jitter 1.9" oldugu degerdi.
# (Tek-engel "gec/ani" kaygisiyla dusurmustum; gercek deneyimde zigzag baskin cikti.)
# Takipci duvar-APF: varsayilan (64px) GEC kaliyordu — cok virajli parkurda carrot
# kose kirpmasiyla birlesince takipciler duvara surtuyordu (pygame sim labirent testi).
# NOT (2026-06-12): takipci duvar-APF'i KALDIRILDI (kullanici karari — Webots
# saflagi: takipciler engel gormez, sadece rotayi izler). Bu iki sabit artik
# KULLANILMIYOR; 'B kalkani' (dokunma esigi itmesi) gerekirse geri gelebilir.
# FOLLOWER_WALL_INFLUENCE = 110.0
# FOLLOWER_WALL_GAIN      = 2.6
# Rota yumusatma (lider ONCEDEN hesaplanan rotayi izler — kullanici istegi):
# gradient-descent + ENGEL-CLAMP: her nokta komsu ortalamasina cekilir ama duvar
# guvenlik payi (WALL_PATH_MARGIN=46) IHLAL EDILEMEZ (clamp) -> asla duvara girmez.
# Cikti SEYREK waypoint'lere (~55px) ornekleniyor; lider MEVCUT kanitli waypoint
# takibiyle izler (LEADER_WP_RADIUS=40 genis gecis). Ilk denemedeki piruet, carrot/
# lookahead + 14px siki yaricap + yogun noktadandi — o yol KULLANILMIYOR.
PATH_SMOOTH_PASSES = 60     # gradient gecis sayisi (hedef secince 1 kez kosulur)
PATH_SMOOTH_WEIGHT = 0.25   # puruzsuzluk kazanci (buyut = daha yuvarlak)
PATH_SMOOTH_DATA   = 0.10   # orijinal rotaya baglilik
PATH_WP_SPACING    = 55.0   # yumusak rotanin waypoint araligi (px)
# NOT: 2 engel arasi bosluktan gecmek icin bosluk > 2*WALL_PATH_MARGIN (~92px) olmali.
# plan_path artik yol bulamayinca margin'i %75 ve %55'e dusurup tekrar dener (YOL PLANLAMA bolumu).

# MINECRAFT-TARZI GRID DUVAR EDITORU (pygame_sim'den birebir port):
# [E] modunda sol tik/surukle = hucre boya, sag tik/surukle = sil; hucreler
# buyuk dikdortgenlere birlestirilir ve HER degisiklikte dosyaya kaydedilir
# (program tekrar acilinca parkur otomatik geri gelir).
GRID = 40   # hucre boyutu (px). Koridor gecisi icin >=3 BOS hucre birak (~120px>92 margin).
PARKUR_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "gercek_parkur.json")
PARKUR_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "parkurlar")

# ====================== FORMASYON ======================
FORMATION_SPACING = 220.0    # runtime'da [o]/[p] tuslariyla degisir -> set_formation_spacing()
# SADECE BOLUNMELI UCGEN (formasyon-degisimi) icin aralik carpani: _build_split_routes
# icindeki peel/birlesme geometrisini (LEAD, kanal-birlestirme) buyutur -> yan robotlar
# birbirine girmeden daha UZAK gecer, carpisma freni stuttering azalir (kullanici:
# 'formasyon gecislerinde biraz daha uzak mesafe'; LINE/PARK/kohezyon ETKILENMEZ).
SPLIT_SPACE_MULT = 1.30   # sim A/B: formasyon-degisimi hard-stop %17->%0, park 3/3, LINE/PARK sabit.
# Sadece peel/birlesme ZAMANLAMASINI buyutur (LEAD); yanal OFF=100 sabit -> kamera FOV
# riski yok. DAR kursta genis LEAD detour'u patlatabilir -> set_target SIGORTASI: split
# kurulamazsa normal aralikla (1.0) TEKRAR dener -> ucgen asla kaybolmaz.
PATH_RECORD_DIST = 4.0
# Path en az 3*FORMATION_SPACING (en arkadaki R2 hedefi) + pay kadar olmali.
# 250 nokta * 4px = 1000px > 3*220=660 -> R2 hedefine ulasabilir.
MAX_PATH_LEN = 250
FORMATION_MORPH_RATE = 200.0   # formasyon degisiminde slot degerlerinin kayma hizi (px/sn)

SETTLE_RADIUS = 60.0         # takipci line slotuna bu kadar yaklasinca "line'a oturdu" sayilir


def _build_formations(spacing):
    return {
        "triangle": {
            1: {"side_offset":  spacing, "fb_offset": 0.0, "target_dist": spacing},
            2: {"side_offset":  0.0,     "fb_offset": 0.0, "target_dist": spacing},
            3: {"side_offset": -spacing, "fb_offset": 0.0, "target_dist": spacing},
        },
        "line": {  # Konvoy sirasi: R3 ONDE (en yavas robot avans alir), R1 orta, R2 arka
            3: {"side_offset": 0.0, "fb_offset": 0.0, "target_dist": 1.0 * spacing},
            1: {"side_offset": 0.0, "fb_offset": 0.0, "target_dist": 2.0 * spacing},
            2: {"side_offset": 0.0, "fb_offset": 0.0, "target_dist": 3.0 * spacing},
        },
    }


FORMATIONS = _build_formations(FORMATION_SPACING)


def set_formation_spacing(new_spacing):
    """[o]/[p] tuslari: araligi degistirir ve FORMATIONS hedef degerlerini gunceller.
    (current_slots morph mekanizmasi yeni hedeflere kademeli kayar.)"""
    global FORMATION_SPACING, FORMATIONS
    FORMATION_SPACING = max(80.0, min(250.0, float(new_spacing)))
    new = _build_formations(FORMATION_SPACING)
    for mode in FORMATIONS:
        for rid in FORMATIONS[mode]:
            FORMATIONS[mode][rid].update(new[mode][rid])
    return FORMATION_SPACING


# Serit-tabanli akilli ACC: formasyona gore serit genisligi (FORMATION_SPACING orani).
# Ucgen genis (yan robotlar), cizgi dar (tek sira).
LANE_WIDTH_RATIO = {"line": 0.5, "triangle": 1.2}

# S-BAZLI KAYAN SLOT (sim portu): line takibinde her takipci, onculunun iz
# uzerindeki konumundan bu oran * FORMATION_SPACING geride kayan slotu hedefler.
# 0.55*220=121px slot araligi — binary fren kesimi ~90px'in ustunde (cakismaz).
# Konvoy uyumu esikleri de bu orana baglidir (rel=+0.15, stp=+0.55).
SLOT_GAP_RATIO = 0.55
# UCGEN COHESION (centroid-offset fuzzy): SADECE bolunmeli ucgen aktifken.
# 'Dikey sanal referans cizgi' = aktif formasyonun AGIRLIK MERKEZI (COM);
# her robot gidis-yonu boyunca COM'a gore +-band icinde tutulur. One firlayan
# apex (onunde robot yok -> line fuzzy'si calismiyordu, R1 78px/s firladi)
# bulanik mantikla frenlenir. band = bu oran * FORMATION_SPACING (~240px).
# (kullanici: 'dikey cizgi + yatay offset', centroid; 2026-06-13 ~1->~2 aralik:
#  daha BELIRGIN/derin ucgen istendi).
TRIANGLE_LEAD_BAND_RATIO = 1.10   # 1.10*220 ~= 242px -> apex COM'dan en fazla ~240px onde (2 aralik)
# KURTARMA MERDIVENI (geri cekil + yan kacis) DEVRE DISI (19:44 logu —
# kullanici karari): kalkis kuyrugunda YANLIS ALARM verip konvoy ortasinda
# geri vites atti (R1 2 kez, pivot %43). Kod duruyor; kosullari sikilastirip
# tekrar acilabilir. Takilanlari eski ilerleme sigortasi yonetir.
STUCK_RECOVERY = False

# =====================================================================
# BULANIK MANTIK (FUZZY) ACC profilleri — Webots Follower_Python.py'den tasindi
# =====================================================================
# Girdi ORANSAL: ratio = onundeki_robot_mesafesi / FORMATION_SPACING.
# Bu sayede o/p tuslari ile FORMATION_SPACING degisse bile fuzzy evreni gecerli kalir.
# Cikti: 0..1 hiz katsayisi (acc_scale). LINE = agresif, TRIANGLE = yumusak.
FUZZY_PROFILES = {
    "line": {  # formasyon araligi (ratio 1.0) TAM HIZ; sadece formasyondan YAKINSA (ratio<0.8) frenle
        "danger":  [0.0, 0.0, 0.35, 0.50],   # trapmf
        "caution": [0.40, 0.60, 0.85],       # trimf  — sadece yakinda yavasla
        "safe":    [0.75, 1.00, 2.50, 2.50], # trapmf — ratio 0.75+ TAM HIZ (formasyonda beklemez)
        "stop":    [0.0, 0.0, 0.20],         # trimf  (cikti)
        "slow":    [0.10, 0.50, 0.80],       # trimf  (cikti)
        "fast":    [0.70, 0.90, 1.00, 1.00], # trapmf (cikti)
        "emergency_ratio": 0.30,             # ~66px: fiziksel carpismadan once dur
    },
    "triangle": {  # yumusak: daha genis marjlar, daha kibar fren
        "danger":  [0.0, 0.0, 0.50, 0.70],
        "caution": [0.60, 0.90, 1.20],
        "safe":    [1.10, 1.50, 2.50, 2.50],
        "stop":    [0.0, 0.10, 0.25],
        "slow":    [0.20, 0.55, 0.85],
        "fast":    [0.75, 0.90, 1.00, 1.00],
        "emergency_ratio": 0.30,   # ~66px: ucgende biraz daha genis guvenli mesafe
    },
}
RATIO_MAX = 2.5

# ====================== PARK ======================
# Park dizilisi (kullanici istegi): HEPSI liderin AZ ARKASINDA tek sira.
# R3 (ilk gelen) tam arkada-ortada, R1 arka-solda, R2 arka-sagda.
# (geri_birim, yan_birim) * PARK_SPACING.  Yan: + sol, - sag.
# Geri birim 0.9 -> 0.7 -> 0.65 (kullanici: "yerimiz kucuk");
# yan 0.8 -> 0.65 -> 0.75 (kullanici: "yanal ofseti biraz arttir"):
# slotlar liderin ~114px arkasinda, yan slotlar merkezden ~131px acikta.
# KESIN ALT SINIR: parkta collision bandi 105px — komsu slot mesafesi
# (min(geri, yan)*175) bandin ustunde kalmali; geri 0.65=114px sinirda.
PARK_OFFSETS = {
    3: (0.65,  0.0),    # R3: liderin hemen arkasi (ilk gelen, ortada)
    1: (0.65, +0.75),   # R1: arka-sol
    2: (0.65, -0.75),   # R2: arka-sag
}


def parse_args(argv=None):
    """Komut satiri override'lari: kamera/port/log hardcode kalmasin."""
    import argparse
    global CAMERA_URL, CAMERA_INDEX, GATEWAY_PORT, LOG_TIMESTAMPED
    p = argparse.ArgumentParser(description="Lider & takipci swarm navigasyon")
    p.add_argument("--camera-url", default=None, help="IP kamera URL (orn. http://ip:8080/video)")
    p.add_argument("--camera-index", type=int, default=None, help="USB kamera indeksi (URL yerine)")
    p.add_argument("--port", default=None, help="Gateway seri portu (orn. COM7)")
    p.add_argument("--flat-log", action="store_true", help="Eski gibi tek nav_log.txt'ye yaz")
    a = p.parse_args(argv)
    if a.camera_index is not None:
        CAMERA_URL, CAMERA_INDEX = "", a.camera_index
    if a.camera_url is not None:
        CAMERA_URL, CAMERA_INDEX = a.camera_url, None
    if a.port is not None:
        GATEWAY_PORT = a.port
    if a.flat_log:
        LOG_TIMESTAMPED = False
    return a

# ===================== GEOMETRI =====================
# GEOMETRI bolumu — Saf geometri yardimcilari (cv2/donanim bagimsiz -> pytest ile test edilir).

def step_value(cur, tgt, max_step):
    """cur degerini tgt'ye en fazla max_step kadar yaklastirir (yumusak gecis)."""
    if abs(tgt - cur) <= max_step:
        return tgt
    return cur + max_step if tgt > cur else cur - max_step


def transform_to_leader_frame(dx, dy, heading_rad):
    """(dx,dy) goreli vektoru lider yon cercevesine cevirir -> (longitudinal, lateral)."""
    c, s = math.cos(heading_rad), math.sin(heading_rad)
    return dx * c + dy * s, -dx * s + dy * c


def seg_intersect(p1, p2, p3, p4):
    """p1-p2 segmenti p3-p4 ile kesisiyor mu? (uc nokta degil, ic kesisim)"""
    def cross2d(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    d1 = cross2d(p3, p4, p1)
    d2 = cross2d(p3, p4, p2)
    d3 = cross2d(p1, p2, p3)
    d4 = cross2d(p1, p2, p4)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False


def pt_seg_dist(p, a, b):
    """p noktasinin a-b segmentine en kisa mesafesi."""
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def seg_seg_dist(p1, p2, p3, p4):
    """Iki segment arasi en kisa mesafe (kesisiyorsa 0)."""
    if seg_intersect(p1, p2, p3, p4):
        return 0.0
    return min(pt_seg_dist(p1, p3, p4), pt_seg_dist(p2, p3, p4),
               pt_seg_dist(p3, p1, p2), pt_seg_dist(p4, p1, p2))


def rect_edges(rect):
    """Dikdortgenin 4 kenar segmentini dondurur."""
    x1, y1, x2, y2 = rect
    return [((x1, y1), (x2, y1)), ((x2, y1), (x2, y2)),
            ((x2, y2), (x1, y2)), ((x1, y2), (x1, y1))]


def pt_in_rect(p, rect, pad=0.0):
    x1, y1, x2, y2 = rect
    return (x1 - pad) <= p[0] <= (x2 + pad) and (y1 - pad) <= p[1] <= (y2 + pad)


def carrot_on_path(fx, fy, path, lookahead):
    """PURE PURSUIT: follower'i path'e projekte eder, lookahead kadar ILERIDE
    (path sonuna=lidere dogru) bir 'havuc' noktasi + o segmentin yonunu dondurur.
    Cizgiyi siki takip ettirir (kose kesmez, salinmaz).
    Doner: (cx, cy, heading_deg) veya None."""
    n = len(path)
    if n < 2:
        return None
    # 1) En yakin segment ve uzerindeki nokta (projeksiyon)
    best_d = 1e18; bi = 0; bt = 0.0
    for i in range(n - 1):
        ax, ay = path[i][0], path[i][1]
        bx, by = path[i + 1][0], path[i + 1][1]
        abx, aby = bx - ax, by - ay
        L2 = abx * abx + aby * aby
        if L2 < 1e-6:
            continue
        t = ((fx - ax) * abx + (fy - ay) * aby) / L2
        t = max(0.0, min(1.0, t))
        cx = ax + t * abx; cy = ay + t * aby
        d = (fx - cx) ** 2 + (fy - cy) ** 2
        if d < best_d:
            best_d = d; bi = i; bt = t
    # 2) Projeksiyondan lookahead kadar ileri yuru
    cx = path[bi][0] + bt * (path[bi + 1][0] - path[bi][0])
    cy = path[bi][1] + bt * (path[bi + 1][1] - path[bi][1])
    rem = lookahead; i = bi
    while i < n - 1:
        nx, ny = path[i + 1][0], path[i + 1][1]
        seg = math.hypot(nx - cx, ny - cy)
        h = math.degrees(math.atan2(path[i + 1][1] - path[i][1],
                                    path[i + 1][0] - path[i][0])) % 360
        if seg >= rem:
            r = (rem / seg) if seg > 1e-6 else 1.0
            return (cx + r * (nx - cx), cy + r * (ny - cy), h)
        rem -= seg; cx, cy = nx, ny; i += 1
    # Path sonu
    h = math.degrees(math.atan2(path[-1][1] - path[-2][1],
                                path[-1][0] - path[-2][0])) % 360
    return (cx, cy, h)


def carrot_locked(fx, fy, path, lookahead, i_hint, window):
    """carrot_on_path'in ILERLEME-KILITLI hali (#1): en yakin segmenti TUM rotada
    DEGIL [i_hint-window, i_hint+window] penceresinde arar (i_hint None ise ilk kez
    tum rotada -> en yakina kilitlenir). Rota kendine yaklassa bile (U-donus, dolanma)
    yanlis segmente snap etmez; robot rotayi SIRAYLA kat eder -> planli rotaya sadik.
    Doner: (cx, cy, heading_deg, seg_i) / None."""
    n = len(path)
    if n < 2:
        return None
    if i_hint is None:
        lo, hi = 0, n - 1
    else:
        lo = max(0, i_hint - window)
        hi = min(n - 1, i_hint + window + 1)
    best_d = 1e18; bi = lo; bt = 0.0
    for i in range(lo, hi):
        ax, ay = path[i][0], path[i][1]
        bx, by = path[i + 1][0], path[i + 1][1]
        abx, aby = bx - ax, by - ay
        L2 = abx * abx + aby * aby
        if L2 < 1e-6:
            continue
        t = max(0.0, min(1.0, ((fx - ax) * abx + (fy - ay) * aby) / L2))
        cx = ax + t * abx; cy = ay + t * aby
        d = (fx - cx) ** 2 + (fy - cy) ** 2
        if d < best_d:
            best_d = d; bi = i; bt = t
    cx = path[bi][0] + bt * (path[bi + 1][0] - path[bi][0])
    cy = path[bi][1] + bt * (path[bi + 1][1] - path[bi][1])
    rem = lookahead; i = bi
    while i < n - 1:
        nx, ny = path[i + 1][0], path[i + 1][1]
        seg = math.hypot(nx - cx, ny - cy)
        h = math.degrees(math.atan2(path[i + 1][1] - path[i][1],
                                    path[i + 1][0] - path[i][0])) % 360
        if seg >= rem:
            r = (rem / seg) if seg > 1e-6 else 1.0
            return (cx + r * (nx - cx), cy + r * (ny - cy), h, bi)
        rem -= seg; cx, cy = nx, ny; i += 1
    h = math.degrees(math.atan2(path[-1][1] - path[-2][1],
                                path[-1][0] - path[-2][0])) % 360
    return (cx, cy, h, bi)


def path_turn_ahead(path, seg_i, dist):
    """path[seg_i]'den itibaren ~dist px ileride biriken TOPLAM |yon degisimi|
    (derece). Yuksek = keskin viraj YAKLASIYOR -> #2: lookahead proaktif kisalsin
    (kose sapmadan once sikilesir)."""
    n = len(path)
    if n < 3 or seg_i >= n - 2 or seg_i < 0:
        return 0.0

    def seg_h(a, b):
        return math.atan2(b[1] - a[1], b[0] - a[0])
    prev = seg_h(path[seg_i], path[seg_i + 1])
    total = 0.0; rem = dist; j = seg_i + 1
    while j < n - 1 and rem > 0:
        h = seg_h(path[j], path[j + 1])
        dd = abs(math.degrees(h - prev)); dd = min(dd, 360 - dd)
        total += dd; prev = h
        rem -= math.hypot(path[j + 1][0] - path[j][0], path[j + 1][1] - path[j][1])
        j += 1
    return total


def project_to_polyline(pt, poly):
    """pt noktasini poly (=[(x,y),...]) cizgisine en yakin noktaya projekte eder.
    Doner: ((px,py), heading_deg) — takipciler liderin gercek izi yerine
    MAVI CIZGI (planlanan rota) uzerinde gider."""
    if not poly or len(poly) < 2:
        return (pt[0], pt[1]), 0.0
    best_d = float("inf")
    best = (pt[0], pt[1])
    best_h = 0.0
    for i in range(len(poly) - 1):
        ax, ay = poly[i]
        bx, by = poly[i + 1]
        abx, aby = bx - ax, by - ay
        L2 = abx * abx + aby * aby
        if L2 < 1e-6:
            continue
        t = ((pt[0] - ax) * abx + (pt[1] - ay) * aby) / L2
        t = max(0.0, min(1.0, t))
        cx, cy = ax + t * abx, ay + t * aby
        d = (pt[0] - cx) ** 2 + (pt[1] - cy) ** 2
        if d < best_d:
            best_d = d
            best = (cx, cy)
            best_h = math.degrees(math.atan2(aby, abx)) % 360
    return best, best_h


def angle_diff_deg(target_deg, current_deg):
    """[-180, 180] araliginda isaretli aci farki."""
    return (target_deg - current_deg + 180) % 360 - 180

# ===================== YOL PLANLAMA =====================
# YOL PLANLAMA bolumu — Yol planlama (visibility graph + Dijkstra), duvar itme (APF)
# ve sert guvenlik (heading_blocked). cv2/donanim bagimsiz -> test edilebilir.

def wall_repulsion(x, y, walls, influence=None, gain=3.2):
    """APF: yakin dikdortgen duvarlardan iten toplam birim-mertebe vektor (rx, ry).
    Robotu duvar kenarindan uzaklastirip etrafindan KAVIS cizdirir (icine girmeyi onler)."""
    if not walls:
        return 0.0, 0.0
    if influence is None:
        influence = ROBOT_RADIUS_PX + 38.0   # ~64px: cizgide minimal, duvara yaklasinca guclu iter
    rx, ry = 0.0, 0.0
    for rect in walls:
        # robotun rect'e en yakin noktasi (kenar/kose)
        cxp = min(max(x, rect[0]), rect[2])
        cyp = min(max(y, rect[1]), rect[3])
        dx, dy = x - cxp, y - cyp
        d = math.hypot(dx, dy)
        if d < 1e-6:
            # icindeyse merkeze gore disari it
            mcx = (rect[0] + rect[2]) * 0.5
            mcy = (rect[1] + rect[3]) * 0.5
            dx, dy = x - mcx, y - mcy
            d = math.hypot(dx, dy) or 1.0
        if d < influence:
            s = gain * (1.0 - d / influence)   # yaklastikca artar (0..gain)
            rx += (dx / d) * s
            ry += (dy / d) * s
    return rx, ry


def wall_clear_dist(x, y, walls):
    """Noktanin en yakin duvara (dikdortgen) merkez uzakligi; duvar yoksa buyuk deger."""
    best = 1e9
    for rect in walls:
        cx = min(max(x, rect[0]), rect[2])
        cy = min(max(y, rect[1]), rect[3])
        d = math.hypot(x - cx, y - cy)
        if d < best:
            best = d
    return best


def heading_blocked(x, y, heading_deg, walls):
    """Robot kendi heading yonunde ilerlerse bir DIKDORTGEN engele
    (ROBOT_RADIUS sisirilmis) girer mi? Girerse True -> ileri hareket bloklanir
    (robot icine ASLA girmez). Cevirme serbest kalir."""
    if not walls:
        return False
    h = math.radians(heading_deg)
    cx_, sy_ = math.cos(h), math.sin(h)
    # heading yonunde birkac mesafede prob (kenar degmeden once blokla)
    r = ROBOT_RADIUS_PX
    for probe in (r, r + 16, r + 34):
        px, py = x + cx_ * probe, y + sy_ * probe
        for rect in walls:
            if pt_in_rect((px, py), rect, r):
                return True
    return False


def path_clear(a, b, walls, margin=0.0):
    """a->b yolu her DIKDORTGEN engelden en az 'margin' px uzakta mi?"""
    for rect in walls:
        if pt_in_rect(a, rect) or pt_in_rect(b, rect):
            return False   # uc nokta kutunun icinde
        for (p3, p4) in rect_edges(rect):
            if seg_seg_dist(a, b, p3, p4) < margin:
                return False
    return True


def _plan_once(start, goal, walls, clearance, path_margin, bounds=None):
    """Tek (clearance, margin) kombinasyonu icin visibility graph + Dijkstra.
    bounds=(w,h) verilirse KARE DISINA tasan kose dugumleri elenir (kamera
    goremedigi yerden rota gecmesin — duvar kare kenarina yakinsa onemli).
    Doner: waypoint listesi (start haric, goal dahil) veya None (yol yok)."""
    nodes = [tuple(start), tuple(goal)]

    # Her dikdortgen engeli clearance kadar DISARI sisir, 4 KOSESINE waypoint koy.
    for (rx1, ry1, rx2, ry2) in walls:
        c = clearance
        for nd in ((rx1 - c, ry1 - c), (rx2 + c, ry1 - c),
                   (rx1 - c, ry2 + c), (rx2 + c, ry2 + c)):
            if bounds is not None:
                if not (0 <= nd[0] <= bounds[0] and 0 <= nd[1] <= bounds[1]):
                    continue   # ekran disi dugum: takip edilemez, rota oradan gecmesin
            nodes.append(nd)

    n = len(nodes)
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if path_clear(nodes[i], nodes[j], walls, margin=path_margin):
                d = math.hypot(nodes[j][0] - nodes[i][0], nodes[j][1] - nodes[i][1])
                adj[i].append((j, d))
                adj[j].append((i, d))

    # Dijkstra: 0=start, 1=goal
    INF = float("inf")
    dist_map = [INF] * n
    dist_map[0] = 0
    prev = [-1] * n
    pq = [(0.0, 0)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist_map[u]:
            continue
        for v, w in adj[u]:
            nd = d + w
            if nd < dist_map[v]:
                dist_map[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    if dist_map[1] == INF:
        return None

    # Yolu geri izleyerek olustur
    path = []
    cur = 1
    while cur != -1:
        path.append(nodes[cur])
        cur = prev[cur]
    path.reverse()
    return path[1:]   # start'i dahil etme


def plan_path(start, goal, walls, clearance=None, bounds=None):
    """
    Visibility graph + Dijkstra ile engellerden kacinan waypoint listesi dondurur.
    Donus: [(x,y), ...] — start haric, goal dahil.
    bounds=(w,h): kare disina tasan kose dugumleri elenir (rota ekrandan cikmaz).

    Tam marginle yol bulunamazsa (orn. robot duvara cok yakin basliyor, veya
    iki engel arasi dar) margin/clearance %75 ve %55'e dusurulup TEKRAR
    denenir. Eski davranis "yol yok -> duz cizgi" idi; bu, robotu duvara surup
    heading_blocked'a guvenmek demekti. Hicbiri tutmazsa yine duz cizgiye duser.
    """
    if clearance is None:
        clearance = WALL_CLEARANCE
    for scale in (1.0, 0.75, 0.55):
        wps = _plan_once(start, goal, walls,
                         clearance=clearance * scale,
                         path_margin=WALL_PATH_MARGIN * scale,
                         bounds=bounds)
        if wps is not None:
            if scale < 1.0:
                print(f"[ROTA] Dar gecit: margin x{scale:.2f} ile yol bulundu "
                      f"({WALL_PATH_MARGIN * scale:.0f}px)")
            return wps
    print("[ROTA] UYARI: Gecerli yol bulunamadi (bosluklar cok dar) -> direkt cizgi")
    return [tuple(goal)]


def _densify_line(points, step):
    """Polyline'a her ~step px'te ara nokta ekler (yumusatma cozunurlugu)."""
    if len(points) < 2:
        return [(float(p[0]), float(p[1])) for p in points]
    out = []
    for i in range(len(points) - 1):
        ax, ay = points[i][0], points[i][1]
        bx, by = points[i + 1][0], points[i + 1][1]
        n = max(1, int(math.hypot(bx - ax, by - ay) / step))
        for k in range(n):
            t = k / n
            out.append((ax + t * (bx - ax), ay + t * (by - ay)))
    out.append((float(points[-1][0]), float(points[-1][1])))
    return out


def _pt_clear_walls(pt, walls, margin):
    for r in walls:
        cx = min(max(pt[0], r[0]), r[2])
        cy = min(max(pt[1], r[1]), r[3])
        if math.hypot(pt[0] - cx, pt[1] - cy) < margin:
            return False
    return True


def _resample(points, spacing):
    """Polyline'i ~spacing araliklarla seyrek waypoint'lere cevirir (son nokta dahil)."""
    if len(points) < 2:
        return [tuple(p) for p in points]
    out = [tuple(points[0])]
    acc = 0.0
    for i in range(1, len(points)):
        acc += math.hypot(points[i][0] - points[i - 1][0],
                          points[i][1] - points[i - 1][1])
        if acc >= spacing:
            out.append(tuple(points[i]))
            acc = 0.0
    if out[-1] != tuple(points[-1]):
        out.append(tuple(points[-1]))
    return out


def smooth_path(points, walls, margin=None):
    """KOSELI rotayi (start..goal) yumusatir; duvar guvenlik payini CLAMP ile KORUR
    (ihlal eden hareket kabul edilmez -> asla duvara yaklasmaz). Uclar sabit.
    Donus: ~PATH_WP_SPACING aralikli SEYREK waypoint listesi (start dahil)."""
    if margin is None:
        margin = WALL_PATH_MARGIN
    raw = [(float(p[0]), float(p[1])) for p in points]
    # NOT (15:29 logu): yogun-waypoint denemesi GERI ALINDI. Yakin (55px) waypoint'e
    # nisan + 200ms kamera gecikmesi -> lider WP'de yerinde pivot kilitlenmesi
    # (%78 pivot, zar zor ilerleme). Temiz hatta UZAK hedefe nisan kararli kalir.
    if len(raw) < 3:
        return _resample(raw, PATH_WP_SPACING)
    dense = _densify_line(raw, 25.0)
    if len(dense) < 3:
        return _resample(dense, PATH_WP_SPACING)
    orig = [list(p) for p in dense]
    new = [list(p) for p in dense]
    a, b = PATH_SMOOTH_DATA, PATH_SMOOTH_WEIGHT
    for _ in range(PATH_SMOOTH_PASSES):
        for i in range(1, len(new) - 1):
            cand = [new[i][d] + a * (orig[i][d] - new[i][d])
                            + b * (new[i - 1][d] + new[i + 1][d] - 2 * new[i][d])
                    for d in (0, 1)]
            if (not walls or (_pt_clear_walls(cand, walls, margin)
                              and path_clear(new[i - 1], cand, walls, margin)
                              and path_clear(cand, new[i + 1], walls, margin))):
                new[i] = cand
    return _resample([tuple(p) for p in new], PATH_WP_SPACING)

# ===================== TAKIP (KALMAN) =====================
# TAKIP bolumu — Robot basina algilama-takip katmani.
#
# Eski sistemde 5 ayri el yapimi mekanizma vardi:
#   pozisyon EMA + aci EMA + hiz EMA + ziplama reddi + oklüzyon tahmini/clamp.
# Bunlarin hepsi tek bir sabit-hiz (CV) Kalman filtresinde birlesti (USE_KALMAN=True):
#   - dt-farkinda: FPS dalgalansa da filtre bant genisligi sabit kalir
#     (eski EMA'da alfa kare-basinaydi, FPS'le degisiyordu).
#   - Ziplama reddi  -> innovation gate (MAX_JUMP_PX ayni anlamda kullanilir).
#   - Hiz tahmini    -> KF durum vektorunden (duplicate-kare bias'i YOK; ana dongu
#     zaten ayni kareyi tekrar islemiyor, bkz. ana dongudeki kare-dedup).
#   - Oklüzyon       -> KF predict + MAX_PREDICT_PX clamp + LOST_TIMEOUT (ayni semantik).
#   - BUGFIX: Uzun kayiptan (TRACK_REINIT_GAP) sonra filtre HAM olcumle SIFIRDAN
#     kurulur. Eski EMA bayat degerle %35/%65 harmanlayip yeniden-yakalama aninda
#     5-8 kare yanlis konum bildiriyordu -> kontrol yanlis yone direksiyon basiyordu.
#
# USE_KALMAN=False ile eski EMA davranisina (reset fix'i dahil) donulebilir.

class AngleFilter:
    """Acilar dairesel oldugu icin sin/cos uzayinda EMA (orijinal yaklasim)."""

    def __init__(self, alpha=None):
        self.alpha = EMA_ALPHA if alpha is None else alpha
        self.sin = None
        self.cos = None

    def reset(self):
        self.sin = None
        self.cos = None

    def update(self, raw_angle):
        rad = math.radians(raw_angle)
        s = math.sin(rad)
        c = math.cos(rad)
        if self.sin is None:
            self.sin = s
            self.cos = c
        else:
            self.sin = self.alpha * s + (1.0 - self.alpha) * self.sin
            self.cos = self.alpha * c + (1.0 - self.alpha) * self.cos
        return math.degrees(math.atan2(self.sin, self.cos)) % 360


class _KalmanCV:
    """Sabit-hiz (constant velocity) 2D Kalman filtresi. Durum: [x, y, vx, vy]."""

    def __init__(self, sigma_acc, sigma_meas):
        self.sa2 = float(sigma_acc) ** 2
        self.R = np.eye(2) * float(sigma_meas) ** 2
        self.H = np.array([[1.0, 0, 0, 0],
                           [0, 1.0, 0, 0]])
        self.x = None   # durum 4-vektor
        self.P = None
        self.t = None   # durumun zamani

    def init(self, zx, zy, t):
        self.x = np.array([zx, zy, 0.0, 0.0], dtype=float)
        # Konum belirsizligi ~olcum, hiz tamamen bilinmiyor (300 px/s std)
        self.P = np.diag([self.R[0, 0], self.R[1, 1], 300.0 ** 2, 300.0 ** 2])
        self.t = t

    def predict_to(self, t):
        dt = t - self.t
        if dt <= 0:
            return
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=float)
        # Beyaz-ivme surec gurultusu (standart piecewise model)
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        Q = self.sa2 * np.array([[dt4 / 4, 0, dt3 / 2, 0],
                                 [0, dt4 / 4, 0, dt3 / 2],
                                 [dt3 / 2, 0, dt2, 0],
                                 [0, dt3 / 2, 0, dt2]], dtype=float)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def correct(self, zx, zy):
        z = np.array([zx, zy], dtype=float)
        y = z - self.H @ self.x                      # innovation
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P


class _EmaFallback:
    """USE_KALMAN=False: orijinal EMA pozisyon + hiz EMA davranisi (reset fix'li)."""

    def __init__(self):
        self.x = None
        self.y = None
        self.vx = 0.0
        self.vy = 0.0
        self.t = None

    def init(self, zx, zy, t):
        self.x, self.y = float(zx), float(zy)
        self.vx = self.vy = 0.0
        self.t = t

    def correct_at(self, zx, zy, t):
        a = EMA_ALPHA
        nx = a * zx + (1.0 - a) * self.x
        ny = a * zy + (1.0 - a) * self.y
        dt = t - self.t
        if 0.0 < dt < LOST_TIMEOUT:
            self.vx = 0.4 * ((nx - self.x) / dt) + 0.6 * self.vx
            self.vy = 0.4 * ((ny - self.y) / dt) + 0.6 * self.vy
        else:
            self.vx = self.vy = 0.0
        self.x, self.y, self.t = nx, ny, t


class RobotTracker:
    """Tek robotun konum/aci/hiz takibi + oklüzyon tahmini.

    Kullanim (her YENI karede):
      ok = tracker.update(raw_x, raw_y, raw_angle, t_frame)  # tag bulunduysa
      est = tracker.estimate(t_frame)  # her robot icin; None -> robot kayip (durur)
    """

    def __init__(self):
        self.kf = _KalmanCV(KF_SIGMA_ACC, KF_SIGMA_MEAS) if USE_KALMAN else None
        self.ema = None if USE_KALMAN else _EmaFallback()
        self.angle_f = AngleFilter()
        self.angle = None          # son filtreli aci
        self.last_meas_t = None    # son KABUL edilen olcumun zamani
        self.last_meas_xy = None   # son kabul edilen olcumdeki (filtreli) konum

    # ---- ic yardimcilar ----
    def _pos_vel(self):
        if USE_KALMAN:
            return float(self.kf.x[0]), float(self.kf.x[1]), float(self.kf.x[2]), float(self.kf.x[3])
        e = self.ema
        return e.x, e.y, e.vx, e.vy

    def _initialized(self):
        return (self.kf.x is not None) if USE_KALMAN else (self.ema.x is not None)

    # ---- API ----
    def update(self, raw_x, raw_y, raw_angle, t):
        """Yeni olcum. Doner: True=kabul, False=ziplama reddi (kare yok sayilir)."""
        gap = None if self.last_meas_t is None else (t - self.last_meas_t)

        # Uzun kayip -> filtreyi HAM olcumle yeniden kur (eski EMA-harman bug'inin fix'i)
        if not self._initialized() or gap is None or gap > TRACK_REINIT_GAP:
            if USE_KALMAN:
                self.kf.init(raw_x, raw_y, t)
            else:
                self.ema.init(raw_x, raw_y, t)
            self.angle_f.reset()
            self.angle = self.angle_f.update(raw_angle)
            self.last_meas_t = t
            self.last_meas_xy = (float(raw_x), float(raw_y))
            return True

        # Ani ziplama kontrolu (innovation gate) — sahte tespit / okluzyon filtresi.
        # Orijinal semantik: son gorusten <0.5 sn gecmisse ve sicrama > MAX_JUMP_PX ise reddet.
        px, py, _, _ = self._pos_vel()
        if gap < 0.5 and math.hypot(raw_x - px, raw_y - py) > MAX_JUMP_PX:
            return False   # bu kareyi reddet, onceki konum korunur

        if USE_KALMAN:
            self.kf.predict_to(t)
            self.kf.correct(raw_x, raw_y)
        else:
            self.ema.correct_at(raw_x, raw_y, t)

        self.angle = self.angle_f.update(raw_angle)
        self.last_meas_t = t
        x, y, _, _ = self._pos_vel()
        self.last_meas_xy = (x, y)
        return True

    def estimate(self, t_now):
        """Robotun su anki en iyi tahmini.
        Doner: dict(x, y, angle, vx, vy, predicted) veya None (kayip -> robot durur).
        predicted=True: bu an icin olcum yok, KF/CV tahmini (LOST_TIMEOUT ve
        MAX_PREDICT_PX ile sinirli — orijinal davranisla birebir)."""
        if self.last_meas_t is None or not self._initialized():
            return None
        gap = t_now - self.last_meas_t
        if gap >= LOST_TIMEOUT:
            return None   # cok uzun kayip -> robot durur (found=False)

        if USE_KALMAN:
            self.kf.predict_to(t_now)
            x, y, vx, vy = self._pos_vel()
        else:
            ex, ey, vx, vy = self._pos_vel()
            x = ex + vx * max(0.0, gap)
            y = ey + vy * max(0.0, gap)

        # Tahminle kacisi sinirla (orijinal MAX_PREDICT_PX clamp'i)
        mx, my = self.last_meas_xy
        dx, dy = x - mx, y - my
        d = math.hypot(dx, dy)
        if gap > 1e-6 and d > MAX_PREDICT_PX:
            s = MAX_PREDICT_PX / d
            x, y = mx + dx * s, my + dy * s

        predicted = gap > 1e-6
        return {"x": x, "y": y, "angle": self.angle,
                "vx": vx, "vy": vy, "predicted": predicted}

    def snapshot(self):
        """collision_scale / nearest_ahead_dist icin 'son gorulen' kaydi
        (orijinal last_known_positions sozlugu ile ayni anlam/format)."""
        if self.last_meas_xy is None:
            return {"x": None, "y": None, "angle": None, "time": 0.0,
                    "vx": 0.0, "vy": 0.0}
        _, _, vx, vy = self._pos_vel() if self._initialized() else (0, 0, 0.0, 0.0)
        return {"x": self.last_meas_xy[0], "y": self.last_meas_xy[1],
                "angle": self.angle, "time": self.last_meas_t,
                "vx": vx, "vy": vy}

    def speed(self):
        """Anlik hiz buyuklugu (px/s) — adaptif lookahead icin."""
        if not self._initialized():
            return 0.0
        _, _, vx, vy = self._pos_vel()
        return math.hypot(vx, vy)

# ===================== KONTROL =====================
# KONTROL bolumu — Motor komutlari (PD), PWM deadband, fuzzy ACC (LUT), serit-ACC,
# oncelik tabanli carpisma yavaslatma ve park slot geometrisi.
# cv2/donanim bagimsiz -> pytest ile test edilir.

# =====================================================================
# FUZZY ACC — egri baslangicta bir kez orneklenip LUT olarak saklanir.
# (skfuzzy compute() yavastir; dongude her kare 3 robot icin cagirmak yerine O(1) arama.)
# =====================================================================

def _build_fuzzy_acc_lut(profile, step=0.02):
    """Bir profil icin fuzzy ACC egrisini orneklenmis (ratios, factors) LUT'a cevirir."""
    r = ctrl.Antecedent(np.arange(0.0, RATIO_MAX + 0.01, 0.01), "ratio")
    f = ctrl.Consequent(np.arange(0.0, 1.01, 0.01), "factor")

    r["danger"]  = fuzz.trapmf(r.universe, profile["danger"])
    r["caution"] = fuzz.trimf(r.universe,  profile["caution"])
    r["safe"]    = fuzz.trapmf(r.universe, profile["safe"])

    f["stop"] = fuzz.trimf(f.universe,  profile["stop"])
    f["slow"] = fuzz.trimf(f.universe,  profile["slow"])
    f["fast"] = fuzz.trapmf(f.universe, profile["fast"])

    rules = [
        ctrl.Rule(r["danger"],  f["stop"]),
        ctrl.Rule(r["caution"], f["slow"]),
        ctrl.Rule(r["safe"],    f["fast"]),
    ]
    sim = ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules))

    ratios = np.arange(0.0, RATIO_MAX + step, step)
    factors = np.empty_like(ratios)
    for i, rv in enumerate(ratios):
        sim.input["ratio"] = float(rv)
        try:
            sim.compute()
            factors[i] = sim.output["factor"]
        except Exception:
            factors[i] = 1.0  # kural tetiklenmezse guvenli taraf: tam hiz
    return ratios, factors


_FUZZY_LUT = {name: _build_fuzzy_acc_lut(prof) for name, prof in FUZZY_PROFILES.items()}


def fuzzy_speed_factor(d_ahead, formation_mode):
    """Onundeki robota olan piksel mesafesine gore bulanik hiz katsayisi (0..1)."""
    if FORMATION_SPACING <= 0:
        return 1.0
    prof = FUZZY_PROFILES.get(formation_mode, FUZZY_PROFILES["line"])
    ratio = d_ahead / FORMATION_SPACING
    if ratio < prof["emergency_ratio"]:
        return 0.0  # acil fren: sert dur
    lut_r, lut_f = _FUZZY_LUT.get(formation_mode, _FUZZY_LUT["line"])
    return float(np.interp(ratio, lut_r, lut_f))


# ---------------------------------------------------------------------
# UCGEN COHESION FUZZY (centroid-offset) — apex'in one firlamasini onler.
# Girdi lead_norm = (robotun gidis-yonu ilerlemesi - formasyon COM'u) / band:
#   <=0  COM'da/gerisinde  -> TAM HIZ (gruba yetis)
#   0..1 one cikiyor       -> kademeli FREN (line ACC ile ayni mantik, ama
#                             girdi 'oone-cikma offset', 'oonundeki bosluk' degil)
#   >=1  band kenari/disi  -> hold (~0.12 crawl; 0 degil -> digerleri ilerleyince
#                             COM gelir, apex serbest kalir, kilitlenme yok)
def _build_cohesion_lut(step=0.02):
    a = ctrl.Antecedent(np.arange(-2.0, 2.01, 0.01), "lead")
    f = ctrl.Consequent(np.arange(0.0, 1.01, 0.01), "factor")
    a["back"] = fuzz.trapmf(a.universe, [-2.0, -2.0, -0.20, 0.15])
    a["edge"] = fuzz.trimf(a.universe,  [0.0, 0.5, 1.0])
    a["over"] = fuzz.trapmf(a.universe, [0.80, 1.10, 2.0, 2.0])
    f["fast"] = fuzz.trapmf(f.universe, [0.80, 0.95, 1.0, 1.0])
    f["slow"] = fuzz.trimf(f.universe,  [0.30, 0.55, 0.85])
    f["hold"] = fuzz.trimf(f.universe,  [0.0, 0.12, 0.30])
    rules = [ctrl.Rule(a["back"], f["fast"]),
             ctrl.Rule(a["edge"], f["slow"]),
             ctrl.Rule(a["over"], f["hold"])]
    sim = ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules))
    xs = np.arange(-2.0, 2.0 + step, step)
    ys = np.empty_like(xs)
    for i, xv in enumerate(xs):
        sim.input["lead"] = float(xv)
        try:
            sim.compute()
            ys[i] = sim.output["factor"]
        except Exception:
            ys[i] = 1.0
    return xs, ys


_COHESION_LUT = _build_cohesion_lut()


def formation_offset_factor(lead_norm):
    """Centroid-offset bulanik hiz katsayisi (~0.12..1.0). Bkz. _build_cohesion_lut."""
    xs, ys = _COHESION_LUT
    return float(np.interp(lead_norm, xs, ys))


# ---------------------------------------------------------------------
# CROSS-TRACK -> LOOKAHEAD FUZZY: rotaya YANAL uzaklik buyudukce pure-pursuit
# lookahead'ini KISALT (carrot yakinda -> rotaya daha DIK/sadik don); rotadayken
# uzun tut (yumusak). 'az saptin hafif kir / cok saptin cok kir' — AYRI bir donus
# terimi DEGIL, pure-pursuit'in KENDI parametresini moduler -> kontrolcuyle kavga
# yok. (Kod zaten 'uzun havuc koseyi kirpiyor' bulgusunu dogruluyor.)
#   Girdi: xte_norm = yanal_hata / XTRACK_REF.  Cikti: lookahead carpani (~0.35..1.0)
def _build_xtrack_lut(step=0.02):
    a = ctrl.Antecedent(np.arange(0.0, 2.51, 0.01), "xte")
    f = ctrl.Consequent(np.arange(0.0, 1.01, 0.01), "factor")
    a["on"]   = fuzz.trapmf(a.universe, [0.0, 0.0, 0.20, 0.50])   # rotada
    a["near"] = fuzz.trimf(a.universe,  [0.30, 0.70, 1.10])       # az sapmis
    a["far"]  = fuzz.trapmf(a.universe, [0.90, 1.30, 2.5, 2.5])   # cok sapmis
    f["long"]  = fuzz.trapmf(f.universe, [0.80, 0.95, 1.0, 1.0])  # uzun lookahead -> yumusak
    f["mid"]   = fuzz.trimf(f.universe,  [0.45, 0.60, 0.80])
    f["short"] = fuzz.trimf(f.universe,  [0.30, 0.35, 0.50])      # kisa -> sert/sadik don
    rules = [ctrl.Rule(a["on"],   f["long"]),
             ctrl.Rule(a["near"], f["mid"]),
             ctrl.Rule(a["far"],  f["short"])]
    sim = ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules))
    xs = np.arange(0.0, 2.5 + step, step)
    ys = np.empty_like(xs)
    for i, xv in enumerate(xs):
        sim.input["xte"] = float(xv)
        try:
            sim.compute()
            ys[i] = sim.output["factor"]
        except Exception:
            ys[i] = 1.0
    return xs, ys


_XTRACK_LUT = _build_xtrack_lut()


def crosstrack_lookahead_factor(xte_norm):
    """Yanal hata (norm) -> lookahead carpani (~0.35..1.0). Bkz. _build_xtrack_lut."""
    xs, ys = _XTRACK_LUT
    return float(np.interp(max(0.0, xte_norm), xs, ys))


# ---------------------------------------------------------------------
# CURVATURE -> LOOKAHEAD FUZZY (#2, PROAKTIF): ileride keskin viraj varsa lookahead'i
# baştan KISALT -> kose KESMEDEN once sikilesir (cross-track reaktif, bu proaktif).
#   Girdi: turn_norm = ileride biriken donus aci / CURV_REF.  Cikti: lookahead carpani
def _build_curv_lut(step=0.02):
    a = ctrl.Antecedent(np.arange(0.0, 2.51, 0.01), "turn")
    f = ctrl.Consequent(np.arange(0.0, 1.01, 0.01), "factor")
    a["straight"] = fuzz.trapmf(a.universe, [0.0, 0.0, 0.25, 0.55])
    a["bend"]     = fuzz.trimf(a.universe,  [0.35, 0.75, 1.15])
    a["sharp"]    = fuzz.trapmf(a.universe, [0.95, 1.35, 2.5, 2.5])
    f["long"]  = fuzz.trapmf(f.universe, [0.80, 0.95, 1.0, 1.0])
    f["mid"]   = fuzz.trimf(f.universe,  [0.45, 0.62, 0.82])
    f["short"] = fuzz.trimf(f.universe,  [0.32, 0.40, 0.55])
    rules = [ctrl.Rule(a["straight"], f["long"]),
             ctrl.Rule(a["bend"],     f["mid"]),
             ctrl.Rule(a["sharp"],    f["short"])]
    sim = ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules))
    xs = np.arange(0.0, 2.5 + step, step)
    ys = np.empty_like(xs)
    for i, xv in enumerate(xs):
        sim.input["turn"] = float(xv)
        try:
            sim.compute()
            ys[i] = sim.output["factor"]
        except Exception:
            ys[i] = 1.0
    return xs, ys


_CURV_LUT = _build_curv_lut()


def curvature_lookahead_factor(turn_norm):
    """Ileri viraj acisi (norm) -> lookahead carpani (~0.35..1.0). Bkz. _build_curv_lut."""
    xs, ys = _CURV_LUT
    return float(np.interp(max(0.0, turn_norm), xs, ys))


def nearest_ahead_dist(fx, fy, my_rid, last_known, now, heading_deg, lane_width, fresh=2.0):
    """Lider yon cercevesinde 'onumde (longitudinal>0) ve seridimde (|lateral|<lane)'
    olan en yakin robotun gercek piksel mesafesi; yoksa inf."""
    heading_rad = math.radians(heading_deg)
    best = float("inf")
    for rid, lkp in last_known.items():
        if rid == my_rid or lkp["x"] is None or (now - lkp["time"]) > fresh:
            continue
        dx, dy = lkp["x"] - fx, lkp["y"] - fy
        lon, lat = transform_to_leader_frame(dx, dy, heading_rad)
        if lon > 0 and abs(lat) < lane_width:
            d = math.hypot(dx, dy)
            if d < best:
                best = d
    return best


# =====================================================================
# MOTOR KOMUTLARI
# =====================================================================

def apply_deadband(val, min_pwm=None):
    """PWM = min_pwm (hareket esigi) + |val|*PWM_SCALE.
    Fuzzy/hiz yumusakligi PWM bandina yansir. Son kuantalama BURADA yapilir;
    oncesinde komutlar float kalmali (int() ile ezilmemeli).
    min_pwm=None -> MIN_PWM (lider). Takipciler FOLLOWER_MIN_PWM ile cagrilir
    (daha dusuk taban = gercekten daha yavas; motor stall'a dikkat)."""
    if min_pwm is None:
        min_pwm = MIN_PWM
    if abs(val) < 1e-6:
        return 0
    sign = 1 if val > 0 else -1
    pwm = sign * (min_pwm + abs(val) * PWM_SCALE)
    return int(max(-MAX_PWM, min(MAX_PWM, pwm)))


_pd_state = {}   # pd_id -> (onceki angle_diff, zaman)
_fwd_state = {}  # pd_id -> (onceki fwd, zaman) — yumusak hizlanma (slew) durumu


def reset_pd():
    """Yeni hedefte cagrilir: eski hedeften kalan turev 'hayaletini' temizler.
    (Eski kodda resetlenmiyordu; hedef degisiminde ilk karede yanlis D katkisi olabiliyordu.)"""
    _pd_state.clear()
    _fwd_state.clear()   # ivme rampasini da sifirla (yeni hedefte sifirdan yumusak kalkis)


def compute_motor_commands(angle_diff, dist, is_catchup=False, speed_scale=1.0,
                           pd_id=None, kd=0.0, kp=0.12, pivot_deg=65.0):
    # pivot_deg 45 -> 65 (16:11 logu): takipciler zamanin %42-61'ini YERINDE
    # PIVOT donerek gecirdi (sag-sol zikzak). 65'e kadar KAVISLE duzeltir —
    # ileri bilesen kesilmez, pivot ancak buyuk hatada devreye girer.
    fwd_limit = FWD_CATCHUP if is_catchup else FWD_MAX
    turn_limit = TURN_CATCHUP if is_catchup else TURN_MAX

    fwd_max = fwd_limit * speed_scale
    turn_max = turn_limit * speed_scale

    abs_angle = abs(angle_diff)
    turn = angle_diff * kp

    # D (turev) sonumlemesi: hata hizla degisiyorsa donusu kis -> overshoot/slalom azalir
    if pd_id is not None and kd > 0.0:
        t_now = time.time()
        prev = _pd_state.get(pd_id)
        if prev is not None:
            dt_pd = t_now - prev[1]
            if dt_pd > 1e-3:
                rate = (angle_diff - prev[0]) / dt_pd   # deg/sn
                turn += kd * rate
        _pd_state[pd_id] = (angle_diff, t_now)

    if dist < SLOW_ZONE:
        ratio = dist / SLOW_ZONE
        ratio = max(0.3, ratio)
        fwd_max *= ratio

    # fwd FLOAT -> fuzzy/hiz yumusakligi korunur, PWM'e apply_deadband yayar.
    # pivot_deg'e kadar YUVARLANARAK doner (kavis); ustunde durup pivot atar.
    if abs_angle > pivot_deg:
        fwd = 0.0
    elif abs_angle > 15.0:
        blend = 1.0 - ((abs_angle - 15.0) / (pivot_deg - 15.0))
        fwd = fwd_max * blend
    else:
        fwd = fwd_max

    # Yumusak hizlanma: fwd'yi zaman-tabanli ivme limitiyle SADECE YUKARI dogru kis.
    # Yavaslama serbest (fren/ACC/carpisma/acil-dur gec kalmaz). turn etkilenmez (cevik donus).
    if SMOOTH_ACCEL and pd_id is not None:
        t_now = time.time()
        pf = _fwd_state.get(pd_id)
        if pf is not None:
            dt_f = t_now - pf[1]
            if dt_f > 1e-3:
                max_up = FWD_ACCEL_LIMIT * dt_f
                if fwd > pf[0] + max_up:
                    fwd = pf[0] + max_up   # hizlanmayi rampala; dususe dokunma
        _fwd_state[pd_id] = (fwd, t_now)

    turn = max(-turn_max, min(turn_max, turn))

    if abs_angle < HEADING_DEADZONE:
        turn = 0.0

    left_cmd = fwd + turn
    right_cmd = fwd - turn
    return left_cmd, right_cmd


def _other_pos(o, robot_states, last_known, now):
    """o robotunun guncel konumu: once canli tespit, yoksa son-bilinen (HAYALET).
    Tag'i kayip robot 0 PWM aldigi icin YERINDE durur -> son konumu COLLISION_GHOST_SEC
    boyunca carpisma icin gecerli sayilir (eski 1s pencere: gorunmez robota toslaniyordu)."""
    s = robot_states.get(o)
    if s and s["found"]:
        return s["x"], s["y"]
    lk = last_known.get(o)
    if not lk or lk["x"] is None or (now - lk["time"]) > COLLISION_GHOST_SEC:
        return None
    return lk["x"], lk["y"]


def robot_repulsion(rid, fx, fy, robot_states, last_known, now,
                    goal_dir=None, influence=None, gain=None):
    """Diger robotlardan iten VORTEKS APF vektoru (rx, ry). Saf radyal itme,
    hedef tam engelin arkasindayken YEREL MINIMUM yaratir (robot ileri-geri
    salinir, gecemez — 19:24 testi R2). Teget bilesen eklenir: robot engele
    carpip geri sekmek yerine HEDEFE YAKIN taraftan kayarak DOLANIR.
    Formasyon araligi (220) > influence (140) -> duzgun takipte etkisi SIFIR."""
    if influence is None:
        influence = ROBOT_REPULSE_DIST
    if gain is None:
        gain = ROBOT_REPULSE_GAIN
    rx, ry = 0.0, 0.0
    for o in ROBOT_IDS:
        if o == rid:
            continue
        pos = _other_pos(o, robot_states, last_known, now)
        if pos is None:
            continue
        dx, dy = fx - pos[0], fy - pos[1]
        d = math.hypot(dx, dy)
        if d < 1e-6 or d >= influence:
            continue
        sc = gain * (1.0 - d / influence)
        ux, uy = dx / d, dy / d              # radyal (engelden uzaga)
        if goal_dir is not None:
            # Teget yon: hedefin engele gore hangi tarafinda kalacagini sec
            # (cross isareti) -> o yana dogru kay; radyal+teget = girdap.
            cz = ux * goal_dir[1] - uy * goal_dir[0]
            if cz >= 0:
                tx_, ty_ = -uy, ux
            else:
                tx_, ty_ = uy, -ux
            rx += (0.6 * ux + 0.8 * tx_) * sc
            ry += (0.6 * uy + 0.8 * ty_) * sc
        else:
            rx += ux * sc
            ry += uy * sc
    return rx, ry


def collision_scale(rid, fx, fy, mvx, mvy, robot_states, last_known, now, stop_dist,
                    anti_ram_skip=None):
    """rid robotu icin YUMUSAK hiz katsayisi (0..1). Iki katman:
      1) ONCELIK katmani: kendisinden YUKSEK oncelikli + on-yari-dairedeki robota
         kademeli yol verir (d>=stop_dist -> 1.0, d<=contact -> 0.0). Sira korunur,
         dusuk oncelikli yol verir -> normal convoy'da deadlock olmaz.
      2) ANTI-RAM katmani: ONCELIK FARKETMEKSIZIN, gidis yonumdeki ('net onumde') ve
         COLLISION_MUTUAL_DIST'ten yakin HER robota fren. Egri/geri-donuslu yolda
         (koridor) slotlar yer degistirip ust-oncelikli alt-onceklinin ustune gidince
         toslamayi onler. Yon konisi -> sadece ustune giden durur (deadlock olmaz)."""
    try:
        my_idx = COLLISION_PRIORITY.index(rid)
    except ValueError:
        return 1.0
    contact = 2.0 * ROBOT_RADIUS_PX + COLLISION_CONTACT_PAD   # ~74px guvenli mesafe
    mv = math.hypot(mvx, mvy)
    higher = set(COLLISION_PRIORITY[:my_idx])
    scale = 1.0
    for o in COLLISION_PRIORITY:
        if o == rid:
            continue
        pos = _other_pos(o, robot_states, last_known, now)
        if pos is None:
            continue
        dx, dy = pos[0] - fx, pos[1] - fy
        d = math.hypot(dx, dy)
        dot = 1.0 if mv <= 1e-3 else (dx * mvx + dy * mvy) / (d * mv) if d > 1e-6 else 1.0

        # 1) ONCELIK katmani: sadece yuksek oncelikliler, KONI-bazli.
        #    ('yakinsa her yon frenle' kurali KALDIRILDI: kacmaya calisan robotu da
        #     kilitliyordu — 19:18 testi R2 donmasi. Uzaklasma her zaman serbest.)
        if o in higher and d < stop_dist:
            ahead = (mv <= 1e-3) or (dot > COLLISION_CONE_DOT)
            away = (mv > 1e-3) and (dot < -0.2)   # net UZAKLASIYOR -> serbest (kacis her zaman acik)
            if (d <= contact and not away) or ahead:
                sc = 0.0 if d <= contact else (d - contact) / (stop_dist - contact)
                scale = min(scale, max(0.0, min(1.0, sc)))

        # 2) ANTI-RAM katmani: oncelik farketmeksizin, net onumdeki yakin robot
        if d < COLLISION_MUTUAL_DIST and dot > COLLISION_MUTUAL_DOT:
            # SIRA-VERME (kullanici: ucgen biterken yanlardan biri beklesin): bu robot
            # anti_ram_skip'teyse (= benden dusuk oncelikli yan robot) GRADED freni atla
            # -> ben (yuksek oncelikli) GECERIM, o oncelik katmaniyla bana yol verir (bekler).
            # TEMAS (<74px) hard-stop HER ZAMAN gecerli kalir -> fiziksel carpisma yok.
            if not (anti_ram_skip and o in anti_ram_skip and d > contact):
                sc = 0.0 if d <= contact else (d - contact) / (COLLISION_MUTUAL_DIST - contact)
                scale = min(scale, max(0.0, min(1.0, sc)))
    # MOTOR GERCEGI: PWM tabani (MIN) yuzunden 0<scale<1 "kademeli fren" fiilen TAM HIZ
    # demek — robot ancak scale TAM 0 olunca durur, sonra ~25px suzulur (18:07 testi:
    # R3, parkli R1'e 49px'e kadar girdi). Dusuk scale'i TAM DUR'a yuvarla.
    # 0.35: komut ~97px'de kesilir, suzulmeyle ~72px'de durur — fiziksel temas 52px'in
    # ustunde ama "cok uzakta durdular" hissi vermez (0.45 fazla cekingen kaliyordu).
    if scale < 0.35:
        scale = 0.0
    return scale


def park_target_pos_safe(rid, lx, ly, langle, walls, spacing=None):
    """park_target_pos + DUVAR DOGRULAMASI: yan slot duvara denk gelirse (veya
    duvara cok yakinsa) MERKEZ hatta tek-sira derin slota duser — dar gecit
    cikisinda park ederken yan slotlar duvar arkasinda kalabiliyor (sim bulgusu)."""
    if spacing is None:
        spacing = PARK_SPACING
    sx, sy = park_target_pos(rid, lx, ly, langle, spacing)
    if wall_clear_dist(sx, sy, walls) >= ROBOT_RADIUS_PX + 25:
        return (sx, sy)
    # Fallback: merkez cizgide tek sira (geri birim 0.65'e uyumlu; slot arasi
    # 0.65*175=114px > park fren bandi 105px — daha sikistirma)
    depth = {3: 0.65, 1: 1.3, 2: 1.95}[rid]
    back_rad = math.radians((langle + 180) % 360)
    fx = lx + depth * spacing * math.cos(back_rad)
    fy = ly + depth * spacing * math.sin(back_rad)
    return (max(45.0, min(CAM_WIDTH - 45.0, fx)),
            max(45.0, min(CAM_HEIGHT - 45.0, fy)))


def park_target_pos(rid, lx, ly, langle, spacing=None):
    """rid takipcisinin SIM-TARZI park hedefi (lider konum/acisina gore dunya koordinati).
    R2 -> lider solu, R3 -> sagi, R1 -> arkasi (PARK_OFFSETS).
    Slot EKRAN ICINE kistirilir: lider kenara yakin varirsa slot ekran disina
    dusuyordu (18:20 testi: R2 slotu x=1366 > 1280) -> robot ulasamayip sigortaya kaliyordu."""
    if spacing is None:
        spacing = PARK_SPACING
    back, side = PARK_OFFSETS[rid]
    back_rad = math.radians((langle + 180) % 360)      # geri yon
    side_rad = math.radians(langle) + (math.pi / 2.0)  # sol yon
    bx = lx + back * spacing * math.cos(back_rad)
    by = ly + back * spacing * math.sin(back_rad)
    px = bx + side * spacing * math.cos(side_rad)
    py = by + side * spacing * math.sin(side_rad)
    pad = 45.0   # robot yaricapi + tag gorunurluk payi
    return (max(pad, min(CAM_WIDTH - pad, px)),
            max(pad, min(CAM_HEIGHT - pad, py)))


def adaptive_lookahead(speed_px_s):
    """Nav2 RPP / MIT Adaptive Pure Pursuit yaklasimi: LA = clamp(t * v, min, max).
    Yavasken siki takip (kose kesmez), hizliyken yumusak (salinmaz)."""
    if not ADAPTIVE_LOOKAHEAD:
        return FOLLOWER_LOOKAHEAD
    la = LOOKAHEAD_TIME * speed_px_s
    return max(LOOKAHEAD_MIN_PX, min(LOOKAHEAD_MAX_PX, la))

# ===================== KAMERA =====================
# KAMERA bolumu — Dusuk gecikmeli kamera okuyucu (arkaplan thread).
#
# YENI guvenlik ozellikleri:
#   - Her kareye monoton artan SEQ numarasi + yakalama zamani (t_frame) eklenir.
#     Ana dongu boylece (a) ayni kareyi iki kez islemez (CPU + hiz-tahmini dogrulugu),
#     (b) kare ESKIDIYSE (Wi-Fi koptu / stream dondu) robotlari guvenli durdurur.
#   - IP stream dusunce otomatik yeniden baglanma denemesi (3 sn'de bir).
#   - stop(): reader thread join edilir, cap okuma ortasindayken release edilmez.


# Backend'ler oncelik sirasi: MSMF once (camera_test ile DroidCam index=2 MSMF'te calisti)
_USB_BACKENDS = [("MSMF", getattr(cv2, "CAP_MSMF", 0)),
                 ("DSHOW", getattr(cv2, "CAP_DSHOW", 0)),
                 ("ANY", 0)]


def _try_open(index, backend, w, h, fps, use_mjpg):
    """Tek bir (backend, mjpg) kombinasyonunu dener; KARE GELIRSE cap dondurur, yoksa None."""
    try:
        cap = cv2.VideoCapture(index, backend) if backend else cv2.VideoCapture(index)
    except Exception:
        return None
    if not cap.isOpened():
        return None
    if use_mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(8):
        r, f = cap.read()
        if r and f is not None:
            rh, rw = f.shape[:2]
            print(f"[KAMERA] index={index} mjpg={use_mjpg} acik={rw}x{rh}")
            return cap
        time.sleep(0.04)
    cap.release()
    return None


def _open_usb_cap(index, w, h, fps):
    """MSMF once (DroidCam USB icin), sonra DSHOW, sonra ANY. MJPG'siz ve MJPG'li dener."""
    for _bname, b in _USB_BACKENDS:
        for mjpg in (False, True):
            cap = _try_open(index, b, w, h, fps, mjpg)
            if cap is not None:
                return cap
    return None


class LatencyFreeCamera:
    """IP kamera veya USB webcam'den dusuk gecikmeli kare okur (arkaplan thread).
    CAMERA_URL doluysa IP stream, bossa CAMERA_INDEX ile USB acilir."""

    RECONNECT_PERIOD = 3.0   # stream dustukten sonra yeniden deneme araligi (sn)

    def __init__(self):
        self.frame = None
        self.seq = 0            # her YENI karede artar (dedup + watchdog icin)
        self.t_frame = 0.0      # son YENI karenin yakalanma zamani
        self.lock = threading.Lock()
        self.running = False
        self.cap = None
        self._thread = None
        self._source = CAMERA_URL if CAMERA_URL else CAMERA_INDEX
        print(f"[KAMERA] Baglaniliyor: {self._source}")

        if not self._open():
            return

        # Ilk kareyi bekle (maks 5 sn)
        for _ in range(50):
            r, f = self.cap.read()
            if r and f is not None:
                rh, rw = f.shape[:2]
                print(f"[OK] Kamera baglandi: {self._source} | {rw}x{rh}")
                with self.lock:
                    self.frame = f
                    self.seq = 1
                    self.t_frame = time.time()
                break
            time.sleep(0.1)
        else:
            print(f"[HATA] Kamera kare vermiyor: {self._source}")
            self.cap.release()
            return

        self.running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _open(self):
        source = self._source
        if isinstance(source, str) and source:
            self.cap = cv2.VideoCapture(source)
        elif isinstance(source, int):
            self.cap = _open_usb_cap(source, CAM_WIDTH, CAM_HEIGHT, CAM_FPS)
        else:
            print("[HATA] CAMERA_URL ve CAMERA_INDEX ikisi de bos!")
            return False
        if self.cap is None or not self.cap.isOpened():
            print(f"[HATA] Kamera acilamadi: {source}")
            return False
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return True

    def _reader(self):
        last_ok = time.time()
        while self.running:
            r, f = (self.cap.read() if self.cap is not None else (False, None))
            if r and f is not None:
                last_ok = time.time()
                with self.lock:
                    self.frame = f
                    self.seq += 1
                    self.t_frame = last_ok
            else:
                time.sleep(0.01)
                # Stream dustu mu? Belirli araliklarla yeniden baglanmayi dene.
                if time.time() - last_ok > self.RECONNECT_PERIOD:
                    print("[KAMERA] Stream koptu, yeniden baglaniliyor...")
                    try:
                        if self.cap is not None:
                            self.cap.release()
                    except Exception:
                        pass
                    self._open()
                    last_ok = time.time()   # bir sonraki denemeye kadar bekle

    def read(self):
        """Doner: (seq, frame_kopyasi_veya_None, t_frame).
        seq degismediyse kare AYNI'dir -> ana dongu islemden kacinir.
        (now - t_frame) > FRAME_STALE_TIMEOUT ise kare BAYAT'tir -> guvenli durdur."""
        with self.lock:
            if self.frame is None:
                return self.seq, None, self.t_frame
            return self.seq, self.frame.copy(), self.t_frame

    def stop(self):
        self.running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)   # cap.read() ortasinda release etme
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass


def load_undistort_maps(calib_file, w, h):
    """camera_calib.npz varsa undistort remap haritalarini dondurur, yoksa (None, None).
    Kalibrasyon farkli cozunurlukte alindiysa ve npz icinde 'image_size' varsa
    K matrisi (fx, fy, cx, cy) otomatik olceklenir; yoksa (w,h) ile alindigi varsayilir."""
    if not os.path.exists(calib_file):
        print(f"[UYARI] Kalibrasyon dosyasi yok ({calib_file}). Undistort atlanacak.")
        return None, None
    try:
        data = np.load(calib_file)
        K = data["camera_matrix"].astype(np.float64).copy()
        dist = data["dist_coeffs"]
        if "image_size" in data:
            cw, ch = [float(v) for v in np.array(data["image_size"]).flatten()[:2]]
            if (int(cw), int(ch)) != (w, h):
                sx, sy = w / cw, h / ch
                K[0, 0] *= sx; K[0, 2] *= sx   # fx, cx
                K[1, 1] *= sy; K[1, 2] *= sy   # fy, cy
                print(f"[KALIB] K matrisi {int(cw)}x{int(ch)} -> {w}x{h} olceklendi")
        newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 0, (w, h))
        map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (w, h), cv2.CV_16SC2)
        print(f"[OK] Kamera kalibrasyonu yuklendi: {calib_file}")
        return map1, map2
    except Exception as e:
        print(f"[UYARI] Kalibrasyon yuklenemedi ({e}). Undistort atlanacak.")
        return None, None

# ===================== SERI HABERLESME =====================
# SERI HABERLESME bolumu — Gateway seri haberlesmesi (ayri TX thread).
#
# Eski tasarimdaki sorunlar ve cozumleri:
#   1) write()/flush() ana donguden cagiriliyordu ve write_timeout YOKTU ->
#      gateway/USB takilirsa TUM kontrol dongusu (watchdog dahil) sonsuza kadar
#      donardi. Simdi: kendi thread'i + write_timeout=SERIAL_WRITE_TIMEOUT.
#   2) Yazma hatalari sessizce yutuluyordu (except: pass). Simdi: sayilir,
#      HUD'da gosterilir, 2 sn'de bir yeniden baglanma denenir.
#   3) PC-ici DEADMAN: ana dongu set_pwms() cagirmayi birakirsa (detector dondu,
#      exception, vb.) CMD_DEADMAN saniye sonra thread KENDILIGINDEN 0 basar.
#      (Robot firmware'inde komut timeout'u yoksa bu, PC calistigi surece son savunmadir.)
#
# Komut formati degismedi: "<id,left,right>\\n", robot basina CMD_PERIOD'da bir,
# PWM 0 olsa bile surekli yayin (firmware timeout'u ileride eklenirse hazir).
# Motor invert kalibrasyonu (ROBOT_CALIBRATION) gonderim aninda uygulanir.

class SerialWriter(threading.Thread):
    def __init__(self, port=None, baud=None):
        super().__init__(daemon=True)
        self.port = port or GATEWAY_PORT
        self.baud = baud or BAUD_RATE
        self.ser = None
        self._lock = threading.Lock()
        self._pwms = {rid: (0, 0) for rid in ROBOT_IDS}
        self._stamp = time.time()
        self._last_sent = {rid: 0.0 for rid in ROBOT_IDS}
        self._running = False
        self._err_count = 0
        self._last_reopen = 0.0
        self.deadman_active = False
        self._open()

    # ---------- baglanti ----------
    def _open(self):
        try:
            self.ser = serial.Serial(
                self.port, self.baud,
                timeout=0.1,
                write_timeout=SERIAL_WRITE_TIMEOUT,  # YENI: yazma asla bloklamaz
            )
            self.ser.dtr = False
            self.ser.rts = False
            print(f"[OK] Seri port acildi: {self.port}")
            return True
        except Exception as e:
            self.ser = None
            print(f"[UYARI] Seri port acilamadi ({e}). Komutlar gonderilmeyecek.")
            return False

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def status_text(self):
        if self.connected:
            return "SERI:OK" if self._err_count == 0 else f"SERI:OK({self._err_count} hata)"
        return "SERI:YOK"

    # ---------- API ----------
    def set_pwms(self, pwms):
        """Ana dongu her islenen karede cagirir. {rid: (l, r)}"""
        with self._lock:
            for rid, lr in pwms.items():
                self._pwms[rid] = (int(lr[0]), int(lr[1]))
            self._stamp = time.time()

    def set_all_zero(self):
        self.set_pwms({rid: (0, 0) for rid in ROBOT_IDS})

    # ---------- TX dongusu ----------
    def run(self):
        self._running = True
        while self._running:
            now = time.time()
            with self._lock:
                pwms = dict(self._pwms)
                stamp = self._stamp
            # PC-ici deadman: ana donguden CMD_DEADMAN suredir guncelleme yoksa 0 bas
            self.deadman_active = (now - stamp) > CMD_DEADMAN
            if self.deadman_active:
                pwms = {rid: (0, 0) for rid in ROBOT_IDS}

            if self.connected:
                for rid in ROBOT_IDS:
                    if now - self._last_sent[rid] >= CMD_PERIOD:
                        self._send(rid, *pwms[rid])
                        self._last_sent[rid] = now
            elif now - self._last_reopen > 2.0:
                self._last_reopen = now
                self._open()
            time.sleep(0.005)

    def _send(self, rid, left_pwm, right_pwm):
        cal = ROBOT_CALIBRATION.get(rid, {})
        if cal.get("left_invert", False):
            left_pwm = -left_pwm
        if cal.get("right_invert", False):
            right_pwm = -right_pwm
        cmd = f"<{rid},{left_pwm},{right_pwm}>\n"
        try:
            self.ser.write(cmd.encode("utf-8"))
        except serial.SerialTimeoutException:
            self._err_count += 1
        except Exception:
            self._err_count += 1
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None   # run() dongusu yeniden baglanmayi dener

    # ---------- guvenli kapanis ----------
    def stop(self):
        """Tum robotlara 3 tur 0 basar, thread'i ve portu kapatir."""
        self._running = False
        try:
            self.join(timeout=1.0)
        except RuntimeError:
            pass   # hic start edilmemis olabilir
        if self.connected:
            try:
                for _ in range(3):
                    for rid in ROBOT_IDS:
                        self.ser.write(f"<{rid},0,0>\n".encode("utf-8"))
                    self.ser.flush()
                    time.sleep(0.05)
                self.ser.close()
                print("[OK] Seri port kapatildi, tum robotlar durduruldu.")
            except Exception as e:
                print(f"[UYARI] Kapanis sirasinda seri hatasi: {e}")

# ===================== GRID PARKUR + FORMASYON PLANI (sim portu) =====================
# pygame_sim.py'de tasarlanip dogrulanan altyapi BIREBIR tasindi (kullanici istegi):
# grid duvar editoru + otomatik parkur kaydi + rota-tabanli formasyon plani +
# rota-bolunmeli ucgen (Webots kanal kurali) + kavsak tam-dur protokolu.

def save_parkur(cells):
    """Cizilen parkuru OTOMATIK kaydeder (her degisiklikte)."""
    try:
        with open(PARKUR_FILE, "w", encoding="utf-8") as f:
            json.dump(sorted(list(cells)), f)
    except OSError:
        pass


def load_parkur():
    """Son parkuru yukler (dosya yoksa bos)."""
    try:
        with open(PARKUR_FILE, encoding="utf-8") as f:
            return set(tuple(c) for c in json.load(f))
    except (OSError, ValueError):
        return set()


def merged_walls(cells):
    """Dolu hucreleri buyuk dikdortgenlere birlestirir (az kose = plan_path hizli/temiz).
    Once satir-ici yatay seritler, sonra ayni genislikteki ardisik satirlar dikeyde."""
    if not cells:
        return []
    by_row = {}
    for (cx, cy) in cells:
        by_row.setdefault(cy, set()).add(cx)
    strips = []
    for cy, xs in by_row.items():
        xs = sorted(xs)
        x0 = prev = xs[0]
        for x in xs[1:]:
            if x == prev + 1:
                prev = x
            else:
                strips.append((x0, prev, cy))
                x0 = prev = x
        strips.append((x0, prev, cy))
    strips.sort(key=lambda t: (t[0], t[1], t[2]))
    used = [False] * len(strips)
    rects = []
    for i, (x0, x1, y0) in enumerate(strips):
        if used[i]:
            continue
        used[i] = True
        y1 = y0
        grew = True
        while grew:
            grew = False
            for j in range(len(strips)):
                if used[j]:
                    continue
                a0, a1, ay = strips[j]
                if a0 == x0 and a1 == x1 and ay == y1 + 1:
                    used[j] = True
                    y1 = ay
                    grew = True
                    break
        rects.append((x0 * GRID, y0 * GRID, (x1 + 1) * GRID, (y1 + 1) * GRID))
    return rects


def route_arc_s(route, x, y):
    """(x,y) noktasinin rota uzerindeki yay-uzunlugu konumu (kavsak protokolu)."""
    bd, s_at, acc = 1e18, 0.0, 0.0
    for i in range(len(route) - 1):
        ax, ay = route[i][0], route[i][1]
        bx, by = route[i + 1][0], route[i + 1][1]
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        if L2 < 1e-9:
            continue
        seglen = math.sqrt(L2)
        t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
        d = (x - (ax + t * vx)) ** 2 + (y - (ay + t * vy)) ** 2
        if d < bd:
            bd, s_at = d, acc + t * seglen
        acc += seglen
    return s_at


class FormationPlan:
    """ROTA-TABANLI FORMASYON PLANI (sim'de dogrulanan kullanici tasarimi):
    Ucgenin amaci PARALEL GECIS — 2 takipci yanlardan (ofset = o noktadaki BOSLUGA
    gore ayarli), 1 takipci (R3) liderin yolundan aradan. Dar yerde LINE'a iner.

    Hesap: rotanin her noktasinda rotaya DIK iki 'sanal biyik' yurutulur, duvara
    carptigi mesafe = o yonun bosluk payi. Webots'un yan IR sensorlerinin harita
    karsiligi — ama TUM rota icin ONCEDEN, deterministik.
    Mantik agaci (Webots kurallarinin karsiligi):
      bosluk < MIN_TRI  -> LINE   (dar gecit girisi)
      bosluk > EXIT_TRI -> TRIANGLE (histerezis: iki esik farkli, titresim yok)
      LINE bolgesi ONE 0.35S cekilir (daralmadan once toparlan) ve
      ARKAYA 0.35S uzatilir (konvoy kuyrugu cikmadan acilma)."""
    MAX_FEEL = 300.0    # biyik max menzili (acik alan)
    SAFETY = 20.0
    MIN_TRI = 85.0      # yan ofset bunun altina duserse ucgen olmaz -> LINE
    EXIT_TRI = 110.0    # tekrar ucgene donus esigi (histerezis)

    def __init__(self, route, walls):
        # Rotayi ~40px'e yogunlastir: hem 2 noktalik duz rotada plan kurulur,
        # hem uzun duzluklerde biyik olcumu yeterli cozunurlukte olur.
        raw = [(float(q[0]), float(q[1])) for q in route]
        pts = []
        for i in range(len(raw) - 1):
            ax, ay = raw[i]
            bx, by = raw[i + 1]
            nseg = max(1, int(math.hypot(bx - ax, by - ay) / 40.0))
            for k in range(nseg):
                t = k / nseg
                pts.append((ax + t * (bx - ax), ay + t * (by - ay)))
        pts.append(raw[-1])
        self.pts = pts
        n = len(self.pts)
        cap = 0.75 * FORMATION_SPACING
        self.cum = [0.0]
        for i in range(1, n):
            self.cum.append(self.cum[-1] + math.hypot(
                self.pts[i][0] - self.pts[i - 1][0],
                self.pts[i][1] - self.pts[i - 1][1]))
        self.normals = []
        offL, offR = [], []
        for i in range(n):
            a = self.pts[max(0, i - 1)]
            b = self.pts[min(n - 1, i + 1)]
            dx, dy = b[0] - a[0], b[1] - a[1]
            mm = math.hypot(dx, dy) or 1.0
            nv = (-dy / mm, dx / mm)
            self.normals.append(nv)
            offL.append(self._feel(self.pts[i], nv, walls))
            offR.append(self._feel(self.pts[i], (-nv[0], -nv[1]), walls))
        # ham biyik -> GUVENLI yan ofset (robot yarisi + pay dus, tavanla sinirla)
        offL = [min(cap, v - ROBOT_RADIUS_PX - self.SAFETY) for v in offL]
        offR = [min(cap, v - ROBOT_RADIUS_PX - self.SAFETY) for v in offR]
        # sivri tekil acilmalari yumusat (komsuluk min penceresi)
        offL = [min(offL[max(0, i - 2):i + 3]) for i in range(n)]
        offR = [min(offR[max(0, i - 2):i + 3]) for i in range(n)]
        # mod tarama (histerezis)
        modes = []
        cur = "triangle"
        for i in range(n):
            lo = min(offL[i], offR[i])
            if cur == "triangle" and lo < self.MIN_TRI:
                cur = "line"
            elif cur == "line" and lo > self.EXIT_TRI:
                cur = "triangle"
            modes.append(cur)
        # LINE bolgelerini one cek + konvoy boyu kadar uzat; kisa ucgen aralarini yut
        S = FORMATION_SPACING
        zones = []
        i = 0
        while i < n:
            if modes[i] == "line":
                j = i
                while j < n and modes[j] == "line":
                    j += 1
                # Kucuk pay yeter: mod PER-ROBOT konumsal sorgulaniyor (Webots'taki
                # global broadcast gecikmesine gerek yok — her robot kendi cikisinda acilir)
                zones.append([self.cum[i] - 0.35 * S, self.cum[j - 1] + 0.35 * S])
                i = j
            else:
                i += 1
        merged = []
        for z in zones:
            if merged and z[0] - merged[-1][1] < 0.6 * S:
                merged[-1][1] = max(merged[-1][1], z[1])
            else:
                merged.append(z)
        self.modes = ["line" if any(z0 <= self.cum[i] <= z1 for z0, z1 in merged)
                      else "triangle" for i in range(n)]
        # LINE icinde ofset 0; gecislerde s-bazli yumusak daralma/acilma (slew)
        for i in range(n):
            if self.modes[i] == "line":
                offL[i] = offR[i] = 0.0
            else:
                offL[i] = max(0.0, offL[i])
                offR[i] = max(0.0, offR[i])
        for arr in (offL, offR):
            for i in range(1, n):
                ds = self.cum[i] - self.cum[i - 1]
                arr[i] = min(arr[i], arr[i - 1] + 0.35 * ds)
            for i in range(n - 2, -1, -1):
                ds = self.cum[i + 1] - self.cum[i]
                arr[i] = min(arr[i], arr[i + 1] + 0.35 * ds)
        self.offL, self.offR = offL, offR

    def _feel(self, q, nv, walls):
        k = 8.0
        while k < self.MAX_FEEL:
            x, y = q[0] + nv[0] * k, q[1] + nv[1] * k
            for r in walls:
                if r[0] <= x <= r[2] and r[1] <= y <= r[3]:
                    return k
            k += 8.0
        return self.MAX_FEEL

    def _locate(self, x, y):
        best, bd = 0, 1e18
        for i, q in enumerate(self.pts):
            d = (q[0] - x) ** 2 + (q[1] - y) ** 2
            if d < bd:
                bd, best = d, i
        return best

    def query(self, rid, x, y):
        """Takipcinin bulundugu rota konumuna gore (mod, imzali_ofset, normal)."""
        i = self._locate(x, y)
        mode = self.modes[i]
        nv = self.normals[i]
        if mode == "line" or rid == 3:      # R3 = merkez (liderin yolundan)
            off = 0.0
        elif rid == 1:                       # R1 = sol serit
            off = +self.offL[i]
        else:                                # R2 = sag serit
            off = -self.offR[i]
        return mode, off, nv

# ===================== GOREV DURUMU =====================
# GOREV DURUMU bolumu — Gorev durumu (hedef, izler, aktivasyon zinciri, park, duvarlar, rota).
#
# Eski koddaki dev mouse_callback'in elle sifirladigi ~20 degisken burada tek
# nesnede toplanir; set_target()/clear_target() tutarli sifirlama garantiler.
#
# NOT (refactor): eski kodda follower_paths {1:[],2:[],3:[]} ucu de AYNI noktayi
# ayni anda aliyordu (uc ozdes kopya). Tek paylasimli iz (follow_trace) yeterli.
# follower_on_path/SETTLE_RADIUS sozlugu de yaziliyor ama hicbir yerde OKUNMUYORDU
# (aktivasyon karari gap mesafesiyle veriliyor) -> kaldirildi.

class Mission:
    def __init__(self):
        # Hedef
        self.target_x = None
        self.target_y = None
        self.target_active = False     # lider varsa bile takipciler bitirene kadar True kalir

        # Izler (deque: MAX_PATH_LEN otomatik, pop(0) maliyeti yok)
        self.leader_path = deque(maxlen=MAX_PATH_LEN)    # (x, y, angle)
        self.follow_trace = deque(maxlen=MAX_PATH_LEN)   # mavi cizgiye projekte iz (x, y, h)
        self.leader_dist_since_target = 0.0

        # Sirali kalkis: Lider -> R1 -> R2 -> R3
        self.follower_activated = {r: False for r in FOLLOWER_IDS}
        self.activation_times = {r: None for r in FOLLOWER_IDS}
        # DINAMIK KALKIS SIRASI: lider varinca rotaya en yakin takipci ILK kalkar
        # (on_leader_arrived hesaplar). Varsayilan = eski sabit sira (fallback).
        self.activation_order = list(FOLLOWER_IDS)
        self.prev_follower = dict(PREV_FOLLOWER)
        # Kalkis tetigi: onceki robotun KENDI KAT ETTIGI yol (aradaki mesafe degil —
        # baslangic dizilimi uzaksa eski tetik R1+R2'yi AYNI ANDA kaldiriyordu).
        self.traveled = {r: 0.0 for r in FOLLOWER_IDS}
        self.last_xy = {r: None for r in FOLLOWER_IDS}

        # Park (sim-tarzi, TAM SIRALI: R2 -> R3 -> R1, tek tek — es zamanli gidince
        # zit slotlara gidenler kafa kafaya gelip karsilikli kilitleniyordu, 18:39 testi)
        self.follower_final_targets = {r: (None, None) for r in FOLLOWER_IDS}
        self.parked = {r: False for r in FOLLOWER_IDS}
        self.park_progress = {}       # rid -> [traveled_o_an, zaman] (ilerleme-bazli sigorta)
        self.stuck = {}               # rid -> kurtarma merdiveni durumu (takilma/kacis)
        self.park_slots = []          # 3 slot konumu (varista hesaplanir)
        self.slot_taken = []          # slot kapildi mi (EN YAKIN BOS slot atamasi)
        self.arrived = {"flag": False, "x": None, "y": None, "angle": None, "time": 0.0}

        # Duvarlar: GRID hucreleri (Minecraft-tarzi editor) -> birlesik dikdortgenler.
        # Son parkur dosyadan OTOMATIK yuklenir (sim portu — "tekrar cizme" istegi).
        self.grid_cells = load_parkur()
        self.walls = merged_walls(self.grid_cells)
        if self.grid_cells:
            print(f"[PARKUR] {len(self.grid_cells)} hucre yuklendi ({PARKUR_FILE})")
        self.wall_mode = False
        self.wall_pending = None
        self.wall_preview = None

        # Planlanan rota
        self.planned_wps = []    # [(x, y), ...] lider waypoint'leri
        self.planned_route = []  # start + wps (takipci projeksiyonu icin tam cizgi)
        self.current_wp = 0
        self.route_calc_t = 0.0

        # ROTA-BOLUNMELI UCGEN + FORMASYON PLANI (sim portu)
        self.fplan = None       # rota-tabanli formasyon plani (bolunme yoksa kurulur)
        self.own_route = {}     # rid -> [(x,y),...] yan robotlarin OZ rotasi (R3/R2)
        self.merge_pts = []     # (x, y, s): yan rotalarin ORTAYA baglandigi kavsaklar
        self.median = []        # sanal orta refuj dikdortgenleri (planlama ici)
        self.split = False      # rota-bolunmeli ucgen aktif mi
        self.whiskers = []      # TANI: kanal karari biyiklari (gozlem cizimi)
        self._fpos = None       # hedef secimi anindaki takipci konumlari (yan atama)
        self.mid_id = 1         # DINAMIK orta: rota merkezine en yakin takipci (varsay. R1)
        self.side_ids = [2, 3]  # diger ikisi YAN (own_route'lu)

    # ------------------------------------------------------------------
    def set_target(self, x, y, start_xy, plan_fn, fpos=None):
        """Sol tik: yeni hedef. Tum gorev durumunu sifirlar, rota planlar.
        plan_fn(start, goal, walls) -> waypoint listesi (plan_path).
        fpos: {rid: (x,y)} takipci konumlari (bolunmede en-yakin yan atamasi)."""
        self.target_x, self.target_y = x, y
        self.target_active = True
        self.leader_path.clear()
        self.follow_trace.clear()
        self.leader_dist_since_target = 0.0

        # YENI AKIS (kullanici istegi): LIDER ONCE TEK BASINA gider, rotayi cizer.
        # Takipciler lider FINISE VARINCA kalkar (on_leader_arrived R3'u baslatir,
        # zincir R3->R1->R2 devam eder) ve liderin izini bastan sona izler.
        self.follower_activated = {1: False, 2: False, 3: False}
        self.activation_times = {1: None, 2: None, 3: None}
        self.traveled = {r: 0.0 for r in FOLLOWER_IDS}
        self.last_xy = {r: None for r in FOLLOWER_IDS}

        self.follower_final_targets = {r: (None, None) for r in FOLLOWER_IDS}
        self.parked = {r: False for r in FOLLOWER_IDS}
        self.park_progress = {}
        self.stuck = {}
        self.park_slots = []          # 3 slot konumu (varista hesaplanir)
        self.slot_taken = []          # slot kapildi mi (EN YAKIN BOS slot atamasi)
        self.arrived = {"flag": False, "x": None, "y": None, "angle": None, "time": 0.0}

        # Bolunme/formasyon plani sifirla (asagida yeniden kurulur)
        self.fplan = None
        self.own_route = {}
        self.merge_pts = []
        self.median = []
        self.split = False
        self.whiskers = []
        self._fpos = fpos
        self.mid_id = 1
        self.side_ids = [2, 3]

        # Rota planla (lider gorunmuyorsa hedefin kendisinden basla — eski davranis)
        self.current_wp = 0
        self.route_calc_t = time.time()
        sx, sy = start_xy if start_xy[0] is not None else (x, y)
        if self.walls:
            raw_wps = plan_fn((sx, sy), (x, y), self.walls)
            # ONCEDEN hesaplanan rota YUMUSATILIR (margin-clamp: duvara asla yaklasmaz);
            # lider seyrek waypoint'leri MEVCUT takip koduyla izler (carrot/lookahead YOK).
            smooth = smooth_path([(sx, sy)] + [tuple(w) for w in raw_wps], self.walls)
            self.planned_wps = smooth[1:]
            print(f"[ROTA] {len(raw_wps)} kose -> {len(self.planned_wps)} yumusak waypoint")
        else:
            self.planned_wps = [(x, y)]   # duvarsiz: uzak hedefe dogrudan (kararli)
        self.planned_route = [(sx, sy)] + [tuple(w) for w in self.planned_wps]
        # SIM PORTU: rota-bolunmeli ucgen (Webots kanal kurali) — kurulamazsa
        # rota-tabanli formasyon plani (LINE/TRIANGLE segmentleri) fallback'i.
        if self.walls and len(self.planned_route) >= 2:
            # KADEMELI SIGORTA: en genis split-araligindan basla; dar kursta detour
            # patlarsa kademeli kucult -> her kurs TASIYABILDIGI en genis araligi alir,
            # ucgen asla kaybolmaz (cok-kanalli/dar kurslarda 1.15, acikta 1.30).
            for _sm in [m for m in (SPLIT_SPACE_MULT, 1.15, 1.0) if m <= SPLIT_SPACE_MULT] or [1.0]:
                self._build_split_routes(space_mult=_sm)
                if self.split:
                    break
            if not self.split:
                self.fplan = FormationPlan(self.planned_route, self.walls)
                nline = sum(1 for md in self.fplan.modes if md == "line")
                print(f"[FORMASYON] plan: {len(self.fplan.modes)} nokta, "
                      f"{nline} LINE / {len(self.fplan.modes) - nline} TRIANGLE")
        # MATLAB export icin planli rotayi dosyaya dok (salt veri, kontrole etkisi yok)
        try:
            with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "gercek_rota.json"), "w", encoding="utf-8") as _rf:
                json.dump([[float(p[0]), float(p[1])] for p in self.planned_route], _rf)
        except OSError:
            pass
        print(f"[HEDEF] Belirlendi: ({x}, {y})")

    def clear_target(self):
        """Sag tik: hedefi ve tum yollari sifirlar (duvarlar KALIR — eski davranis)."""
        self.target_x = None
        self.target_y = None
        self.target_active = False
        self.leader_path.clear()
        self.follow_trace.clear()
        self.leader_dist_since_target = 0.0
        self.follower_activated = {r: False for r in FOLLOWER_IDS}
        self.activation_times = {r: None for r in FOLLOWER_IDS}
        self.traveled = {r: 0.0 for r in FOLLOWER_IDS}
        self.last_xy = {r: None for r in FOLLOWER_IDS}
        self.follower_final_targets = {r: (None, None) for r in FOLLOWER_IDS}
        self.parked = {r: False for r in FOLLOWER_IDS}
        self.park_progress = {}
        self.stuck = {}
        self.park_slots = []          # 3 slot konumu (varista hesaplanir)
        self.slot_taken = []          # slot kapildi mi (EN YAKIN BOS slot atamasi)
        self.arrived = {"flag": False, "x": None, "y": None, "angle": None, "time": 0.0}
        self.planned_wps = []
        self.planned_route = []
        self.current_wp = 0
        self.fplan = None
        self.own_route = {}
        self.merge_pts = []
        self.median = []
        self.split = False
        print("[HEDEF] Temizlendi ve tum yollar sifirlandi.")

    def clear_walls_and_route(self):
        """[C] tusu: duvarlari (grid hucreleri dahil) ve plani temizler."""
        self.grid_cells.clear()
        save_parkur(self.grid_cells)
        self.walls.clear()
        self.wall_pending = None
        self.planned_wps = []
        self.current_wp = 0
        self.fplan = None
        self.own_route = {}
        self.merge_pts = []
        self.median = []
        self.split = False
        print("[DUVAR] Tum duvarlar temizlendi (parkur dosyasi bosaltildi).")

    # ------------------------------------------------------------------
    def _build_split_routes(self, space_mult=None):
        """BOLGESEL (segmental) bolunme — Webots'un gercek davranisi (sim portu):
        Rotadaki HER iki-yanli KANAL icin ayri karar verilir. Kanal civarinda
        yan robotlar kanalin engellerini DISARIDAN dolanir ve kanal bitince
        ORTA rotaya GERI BIRLESIR; dolanmasi imkansiz kanal SADECE o bolgede
        tek-sira gecilir. R1 daima ortadan. Yan atamasi EN YAKIN takipciye."""
        S = FORMATION_SPACING * (space_mult if space_mult is not None
                                 else SPLIT_SPACE_MULT)   # SADECE split peel/birlesme araligi
        self.own_route = {}  # retry guvenligi: bos basla (sigorta ikinci kez cagirabilir)
        self.merge_pts = []
        self.whiskers = []   # TANI: kanal karari biyiklari (gozlem cizimi icin)
        raw = self.planned_route
        dense, cum = [], [0.0]
        for i in range(len(raw) - 1):
            ax, ay = raw[i][0], raw[i][1]
            bx, by = raw[i + 1][0], raw[i + 1][1]
            nseg = max(1, int(math.hypot(bx - ax, by - ay) / 12.0))  # 25->12: kucuk (1x1) engel cozunurlugu
            for k in range(nseg):
                t = k / nseg
                dense.append((ax + t * (bx - ax), ay + t * (by - ay)))
        dense.append((float(raw[-1][0]), float(raw[-1][1])))
        for i in range(1, len(dense)):
            cum.append(cum[-1] + math.hypot(dense[i][0] - dense[i - 1][0],
                                            dense[i][1] - dense[i - 1][1]))
        total = cum[-1]
        n = len(dense)
        CH_T = 220.0   # 'yan kapali' esigi — Webots NARROW_THRESHOLD analogu

        def _feel2(q, nv):
            k = 8.0
            while k < 300.0:
                x, y = q[0] + nv[0] * k, q[1] + nv[1] * k
                for r in self.walls:
                    if r[0] <= x <= r[2] and r[1] <= y <= r[3]:
                        return k
                k += 8.0
            return 300.0

        normals, leftHit, rightHit = [], [], []
        for i in range(n):
            a = dense[max(0, i - 1)]
            b = dense[min(n - 1, i + 1)]
            dxn, dyn = b[0] - a[0], b[1] - a[1]
            mm = math.hypot(dxn, dyn) or 1.0
            nv = (-dyn / mm, dxn / mm)
            normals.append(nv)
            fL_ = _feel2(dense[i], nv)
            fR_ = _feel2(dense[i], (-nv[0], -nv[1]))
            leftHit.append(fL_ < CH_T)
            rightHit.append(fR_ < CH_T)
            if i % 3 == 0:   # TANI: kanal karari biyiklari (gozlem)
                self.whiskers.append((dense[i][0], dense[i][1],
                                      nv[0], nv[1], fL_, fR_))
        # SAGLAM kanal: ayni noktada iki yan kapali sarti yerine, bir PENCERE icinde
        # SOLDA da SAGDA da engel olmasi yeter -> engeller x'te kaysa veya rota egik
        # olsa bile tetiklenir (eski 'ayni nokta' sarti gercek yerlesimde kaciyordu).
        win = max(1, int(50.0 / 12.0))   # +-50px pencere (12px ornekleme); kayma/egikligi
        # kopruler ama uzak kanallari BIRLESTIRMEZ (80px SELFTEST-5'te 2 kanali birlestirdi)
        both = []
        for i in range(n):
            lo, hi = max(0, i - win), min(n, i + win + 1)
            both.append(any(leftHit[lo:hi]) and any(rightHit[lo:hi]))
        # kanal araliklari (KISA engel de ucgeni hak eder: >=35px)
        chans = []
        i = 0
        while i < n:
            if both[i]:
                j = i
                while j < n and both[j]:
                    j += 1
                if cum[j - 1] - cum[i] >= 18.0:   # 35->18: KUCUK (1x1=40px) engel de kanal sayilir
                    chans.append((i, j - 1))
                i = j
            else:
                i += 1
        # Kanal TESPITI dilated 'both' ile (kayma/egiklige dayanikli) AMA refuj/detour
        # icin GERCEK engel araligina TRIM et: dilated extent fazla genis -> refuj uzar,
        # detour patlardi (SELFTEST-5 kanal1). Gercek araliga kisinca tight ve dolanilir.
        chans = [((lambda r: (r[0], r[-1]) if r else (c0, c1))(
                  [k for k in range(c0, c1 + 1) if leftHit[k] or rightHit[k]]))
                 for (c0, c1) in chans]
        merged = []
        for c in chans:
            if merged and cum[c[0]] - cum[merged[-1][1]] < 0.45 * S:
                merged[-1] = (merged[-1][0], c[1])
            else:
                merged.append(c)
        if not merged:
            self.split = False
            print("[BOLUNME] Iki yanli KANAL yok (tek engel/acik alan) -> LINE takip")
            return

        LEAD = 0.8 * S

        def idx_at(sv):
            sv = max(0.0, min(total, sv))
            lo = 0
            while lo < n - 1 and cum[lo + 1] < sv:
                lo += 1
            return lo

        self.median = []
        detours = {+1.0: [], -1.0: []}   # yan (isaret) -> dolanmalar; atama SONRA yakinliga gore
        first_pin = {}
        park_lim = total - PARK_ENTER_DIST * 0.9
        for (c0, c1) in merged:
            ie = idx_at(cum[c0] - LEAD)
            # Birlesme noktasi park bolgesine tasarsa kanal icine SIKISTIRMA —
            # dolanma dogrudan FINISE baglanir (orta rotayla finiste bulusur).
            to_goal = (cum[c1] + LEAD) > park_lim
            ix = (n - 1) if to_goal else idx_at(cum[c1] + LEAD)
            if ix - ie < 3:
                continue
            if not to_goal:
                self.merge_pts.append((dense[ix][0], dense[ix][1], cum[ix]))
            med_hi = min(ix, idx_at(park_lim))   # refuj park bolgesini kapatmaz
            # SISMAN refuj (yarim 40px): ince refujde planlayici kanal genisse
            # yarim-kanaldan SIZIYORDU (dolanmiyordu) — simdi sizma imkansiz.
            med = [(dense[j][0] - 40, dense[j][1] - 40, dense[j][0] + 40, dense[j][1] + 40)
                   for j in range(ie, med_hi + 1, 2)]
            self.median += med
            vwalls = list(self.walls) + med
            for sgn in (+1.0, -1.0):
                nv_e, nv_x = normals[ie], normals[ix]
                OFF = 100.0   # refuj yarisi(40) + marj(46) + pay
                p_in = (dense[ie][0] + sgn * nv_e[0] * OFF,
                        dense[ie][1] + sgn * nv_e[1] * OFF)
                if to_goal:
                    p_out = (dense[-1][0], dense[-1][1])
                else:
                    p_out = (dense[ix][0] + sgn * nv_x[0] * OFF,
                             dense[ix][1] + sgn * nv_x[1] * OFF)
                wps = _plan_once(p_in, p_out, vwalls, clearance=WALL_CLEARANCE,
                                 path_margin=WALL_PATH_MARGIN,
                                 bounds=(CAM_WIDTH, CAM_HEIGHT))
                if wps is None:
                    print("[BOLUNME] kanal@%d-%dpx: %s yan dolanma YOK -> o bolge tek-sira"
                          % (cum[c0], cum[c1], '+' if sgn > 0 else '-'))
                    continue
                sm = smooth_path([p_in] + [tuple(q) for q in wps], vwalls)
                detours[sgn].append((ie, ix, [tuple(q) for q in sm]))
                first_pin.setdefault(sgn, p_in)
        if not detours[+1.0] and not detours[-1.0]:
            self.split = False
            self.median = []
            self.merge_pts = []
            print("[BOLUNME] Hicbir kanal dolanamadi -> LINE takip")
            return
        # DINAMIK ORTA (kullanici: 'ortaya yakin olan ortaya gecsin, az statik'):
        # rota MERKEZ cizgisine en yakin takipci ortadan gecer (own_route YOK -> izi
        # takip eder); diger ikisi YAN. Tum takipci konumlari bilinmiyorsa eski
        # varsayilan (R1 orta) korunur -> headless/eksik-fpos durumlari guvenli.
        fp = self._fpos or {}
        all_fp = all(fp.get(r) and fp[r][0] is not None for r in FOLLOWER_IDS)
        if all_fp:
            def _d2route(rid):
                fx, fy = fp[rid][0], fp[rid][1]
                return min(math.hypot(fx - dx, fy - dy) for (dx, dy) in dense)
            self.mid_id = min(FOLLOWER_IDS, key=_d2route)
        else:
            self.mid_id = 1
        self.side_ids = sorted(r for r in FOLLOWER_IDS if r != self.mid_id)
        a, b = self.side_ids
        # YAN ATAMA YAKINLIGA GORE: iki esleme karsilastirilir, toplam giris
        # mesafesi kucuk olan kazanir (kullanici kurali: en kolay rota onun).
        ok_fp = all(fp.get(r) and fp[r][0] is not None for r in self.side_ids)
        if ok_fp:
            ref = {sgn: first_pin.get(sgn, dense[len(dense) // 3])
                   for sgn in (+1.0, -1.0)}

            def _d(rid, sgn):
                q = ref[sgn]
                return math.hypot(fp[rid][0] - q[0], fp[rid][1] - q[1])
            if _d(a, -1.0) + _d(b, +1.0) <= _d(a, +1.0) + _d(b, -1.0):
                pair = {a: -1.0, b: +1.0}
            else:
                pair = {a: +1.0, b: -1.0}
        else:
            pair = {a: -1.0, b: +1.0}   # konum yoksa varsayilan (kucuk-id ust)
        # KADEMELI BIRLESME: iki yan ayni orta noktaya ayni anda baglanirsa
        # kafa kafaya kilitleniyorlar (sim bulgusu). Ikinci yan (side_ids[1]) ortaya
        # ~2 nokta DAHA GEC baglanir -> dogal olarak digerinin ARKASINA takilir.
        for rid in self.side_ids:
            lag = 2 if rid == self.side_ids[1] else 0
            pts, cur = [], 0
            for (ie, ix, det) in sorted(detours[pair[rid]]):
                ix2 = min(n - 1, ix + lag)
                pts += dense[cur:ie + 1]
                pts += det
                if lag and ix2 > ix:
                    pts.append(dense[ix2])
                cur = ix2
            pts += dense[cur:]
            self.own_route[rid] = pts
        self.split = True
        print("[BOLUNME] BOLGESEL ucgen: %d kanal | ORTA=R%d (dinamik) | "
              "R%d=%s yan (%d dolanma), R%d=%s yan (%d dolanma)"
              % (len(merged), self.mid_id,
                 a, '+' if pair[a] > 0 else '-', len(detours[pair[a]]),
                 b, '+' if pair[b] > 0 else '-', len(detours[pair[b]])))

    # ------------------------------------------------------------------
    def _activation_order(self, robot_states):
        """Takipcileri liderin IZINE (follow_trace) en yakindan uzaga sirala —
        fiziksel dizilise gore ROTAYA EN YAKIN takipci ILK kalkar. Boylece
        line dizilisinde EN ONDEKI (lidere yakin) once kalkar, sabit R3-bas
        artik en arkadakini ilk kaldirmaz. Konumu olmayan robot sona;
        iz/konum yoksa eski sabit sira [3, 1, 2]."""
        tr = self.follow_trace
        if not tr or not robot_states:
            return [3, 1, 2]
        sample = [(p[0], p[1]) for p in tr][::5] or [(p[0], p[1]) for p in tr]
        BIG = float("inf")

        def d2route(rid):
            st = robot_states.get(rid)
            if not st or not st.get("found"):
                return BIG
            fx, fy = st["x"], st["y"]
            return min((fx - qx) ** 2 + (fy - qy) ** 2 for qx, qy in sample)
        return sorted(FOLLOWER_IDS, key=d2route)

    def on_leader_arrived(self, lx, ly, langle, now, robot_states=None):
        """Lider finise vardi (YENI AKIS): TAKIPCILER SIMDI baslar.
        Rotaya EN YAKIN takipci hemen kalkar; zincir (prev_follower, traveled
        tetigi) digerlerini yakindan uzaga sirayla kaldirir. Hepsi liderin
        cizdigi izi BASTAN SONA izler; finise PARK_ENTER_DIST kadar yaklasan
        PARK pozisyonuna gecer (R3 sag, R2 sol, R1 arka — follower dongusunde)."""
        self.arrived["flag"] = True
        # PARK YONU SABIT YATAY (sim portu — kullanici istegi): liderin finisteki
        # burnu nereye bakarsa baksin slotlar YATAY dizilir — lider 90 derece donuk
        # bitirse bile park duzeni sapmaz. 'Arka' = genel gidis yonunun tersi
        # (soldan-saga kosularda slotlar liderin BATISINDA; sagdan-sola otomatik ayna).
        tr = self.follow_trace
        langle = 0.0
        if len(tr) >= 2 and (tr[-1][0] - tr[0][0]) < 0:
            langle = 180.0
        self.arrived["x"], self.arrived["y"], self.arrived["angle"] = lx, ly, langle
        self.arrived["time"] = now
        # SLOT HAVUZU: konumlar sabit ama atama SABIT DEGIL — park kapisina gelen
        # EN YAKIN BOS slotu kapar (capraz gidisler/kesisme carpismalari onlenir).
        self.park_slots = [park_target_pos_safe(r, lx, ly, langle, self.walls)
                           for r in (3, 1, 2)]
        self.slot_taken = [False, False, False]
        # KALKIS SIRASI:
        #  - BOLUNMELI UCGEN: YANLAR (R2,R3) HEMEN kalkar, ORTA (R1) GECIKMELI
        #    (R1<-R3 zinciri: R3 ~1/3 aralik yol alinca). Orta yanlari bekler ->
        #    ucgen duzgun olusur (kullanici: 'ortadaki digerlerini beklesin').
        #    [Dinamik sira split'te ortayi bir yandan ONCE kaldiriyordu -> bozuktu.]
        #  - LINE: rotaya EN YAKIN takipci ILK (dinamik), zincir yakindan uzaga.
        if self.split:
            mid = getattr(self, "mid_id", 1)
            sides = list(getattr(self, "side_ids", [2, 3]))
            # GERCEK LOG (191906, k96): iki yan AYNI ANDA kalkip baslangicta ~40px'e
            # girince KARSILIKLI CARPISMA kilidi (~18 kare takili). COZUM: gidis yonunde
            # ONDE olan yan ONCE kalkar, ARKADAKI ~0.5 aralik acilinca takip eder
            # (SIRA-VERME, arkadan ramlamaz), ORTA en son. [sim proximity metrigi
            # yaniltmisti; gercek deadlock latency-kaynakli -> log delili esas alindi.]
            if len(tr) >= 2 and robot_states:
                ddx, ddy = tr[-1][0] - tr[0][0], tr[-1][1] - tr[0][1]
                mg = math.hypot(ddx, ddy) or 1.0
                ux, uy = ddx / mg, ddy / mg

                def _proj(r):
                    st = robot_states.get(r)
                    if not st or not st.get("found") or st.get("x") is None:
                        return -1e9
                    return st["x"] * ux + st["y"] * uy
                sides.sort(key=_proj, reverse=True)   # gidis yonunde ONDE olan once
            self.side_ids = sides
            self.activation_order = sides + [mid]
            self.prev_follower = {sides[1]: sides[0], mid: sides[1]}
            self.follower_activated[sides[0]] = True   # SADECE onde olan yan hemen
            self.activation_times[sides[0]] = now
            print("[OK] Finise ulasildi! BOLUNMELI ucgen: ONDE yan (R%d) kalkti; "
                  "ARKA yan (R%d) ve ORTA (R%d) sirayla (aralikli) katilir."
                  % (sides[0], sides[1], mid))
        else:
            order = self._activation_order(robot_states)
            self.activation_order = order
            self.prev_follower = {order[i]: order[i - 1] for i in range(1, len(order))}
            head = order[0]
            self.follower_activated[head] = True
            self.activation_times[head] = now
            print("[OK] Finise ulasildi! iz takibi — kalkis sirasi (rotaya yakinlik): "
                  "R%d -> R%d -> R%d." % (order[0], order[1], order[2]))
        # Lider hedefi temizlenir; target_active TRUE kalir (takipciler devam eder)
        self.target_x = None
        self.target_y = None

# ===================== ARAYUZ / CIZIM =====================
# ARAYUZ bolumu — Cizim (overlay/HUD), fare callback'i ve durum gosterimi.
# Gorsel davranis orijinalle ayni; eklenenler: seri/kamera durum rozetleri ve
# kamera donunca tam ekran kirmizi guvenlik banner'i.

ROBOT_COLORS = {
    0: ((0, 255, 0),   "LIDER"),
    1: ((255, 255, 0), "T1"),
    2: ((0, 255, 255), "T2"),
    3: ((255, 0, 255), "T3"),
}
FOLLOWER_TARGET_COLORS = {1: (255, 255, 0), 2: (0, 255, 255), 3: (255, 0, 255)}

# ===================== TANI KATMANI (sim portu — salt gozlem) =====================
# follower bloku her karede DBG[rid]'e 'neden boyle davrandigini' yazar; cizim ve
# log bunu okur. KONTROL KODUNA ETKISI YOK (yalniz okunan alanlar yazilir).
# state metni: TAKIP / CATCHUP / FUZZY %n / COHESION / KAVSAK BEKLE / SLOT TUT /
#              CARPISMA FRENI / FREN %n / KURTARMA / PARKA GIT / PARK / BEKLE
DBG = {1: {}, 2: {}, 3: {}}
RULER = {"a": None, "b": None, "live": False}   # orta-tik cetvel (px + cm)
STATE_COLORS = {   # rozet/gantt renkleri (BGR — cv2)
    "KURTARMA": (60, 60, 255), "CARPISMA": (70, 70, 240), "FREN": (70, 110, 240),
    "KAVSAK": (200, 90, 230), "COHESION": (60, 160, 240), "FUZZY": (80, 215, 250),
    "SLOT": (165, 150, 150), "CATCHUP": (235, 165, 70), "PARKA": (200, 220, 120),
    "PARK": (115, 100, 100), "BEKLE": (84, 70, 70), "TAKIP": (115, 200, 95),
}


def state_color(st):
    for pre, col in STATE_COLORS.items():
        if str(st).startswith(pre):
            return col
    return (140, 130, 130)


# ===================== TANI PENCERELERI (sim portu — [T] baslatici) =====================
# cv2'de native pencere yok -> overlay olarak cizilir, fare callback'i ile suruklenir.
# Hepsi SALT-GOZLEM. WINDOWS: per-pencere {open,x,y,drag}. SEL_ROBOT: INCELE hedefi.
SEL_ROBOT = {"rid": 3}
WIN_DEFS = {"RAPOR": (372, 168), "INCELE": (312, 168), "HIZ": (340, 150),
            "PWM": (300, 150), "FUZZY": (372, 168), "SENARYO": (392, 0),
            "AYAR": (352, 120)}
WIN_ORDER = list(WIN_DEFS)
WINDOWS = {}
for _i, _n in enumerate(WIN_DEFS):
    WINDOWS[_n] = {"open": False, "x": 40 + (_i % 3) * 400,
                   "y": 110 + (_i // 3) * 200, "drag": None}
LAUNCHER = {"open": False}
LAUNCH_Y = 84
SPDH = deque(maxlen=120)    # (t, v1, v2, v3) hiz gecmisi (HIZ penceresi)
LAST_CMDS = {0: (0, 0), 1: (0, 0), 2: (0, 0), 3: (0, 0)}
REPORT = {"cur": None, "prev": None}
RUN = {"key": None, "t0": 0.0, "t_arr": None, "done": True,
       "piv": {1: 0, 2: 0, 3: 0}, "idle": {1: 0, 2: 0, 3: 0},
       "tot": {1: 0, 2: 0, 3: 0}, "d31": [], "d12": [], "ldev": []}
AYAR_ROWS = [("Collision KUCUK (60px)", 8.0),
             ("Collision NORMAL (74px)", 22.0)]


def list_parkurlar():
    import glob as _g
    try:
        return sorted(_g.glob(os.path.join(PARKUR_DIR, "*.json")))
    except OSError:
        return []


def save_named_parkur(cells):
    try:
        os.makedirs(PARKUR_DIR, exist_ok=True)
        fn = os.path.join(PARKUR_DIR, time.strftime("parkur_%Y%m%d_%H%M%S.json"))
        with open(fn, "w", encoding="utf-8") as f:
            json.dump(sorted(list(cells)), f)
        return fn
    except OSError:
        return None


_SCN_CACHE = {"rows": None, "t": 0.0}


def get_scn_rows():
    # CACHE: disk okumasi kare basina 3x cagriliyordu (win_h+draw+hit) -> 1sn
    # onbellek. Kaydet/sil sonrasi _SCN_CACHE['t']=0 ile zorla tazelenir.
    if _SCN_CACHE["rows"] is not None and time.time() - _SCN_CACHE["t"] < 1.0:
        return _SCN_CACHE["rows"]
    rows = [("save", "[ KAYDET: mevcut cizimi dosyaya ]", None)]
    for f in list_parkurlar():
        try:
            n = len(json.load(open(f, encoding="utf-8")))
        except Exception:
            n = -1
        rows.append(("file", "%s (%d hucre)" % (os.path.basename(f)[:-5], n), f))
    _SCN_CACHE["rows"] = rows
    _SCN_CACHE["t"] = time.time()
    return rows


SCN_MAX_VIS = 9   # SENARYO penceresinde ayni anda gorunen max satir (kaydirmali)


def scn_scroll_max():
    return max(0, len(get_scn_rows()) - SCN_MAX_VIS)


def win_h(name):
    if name == "SENARYO":
        vis = min(len(get_scn_rows()), SCN_MAX_VIS)
        return 24 + vis * 22 + 8   # baslik + gorunen satirlar + kaydirma ipucu
    return WIN_DEFS[name][1]


def launcher_hit(x, y):
    if not LAUNCHER["open"] or not (LAUNCH_Y <= y <= LAUNCH_Y + 18):
        return None
    for i, n in enumerate(WIN_DEFS):
        if 8 + i * 78 <= x <= 8 + i * 78 + 74:
            return n
    return None


def windows_hit(x, y):
    """En ustteki acik pencereden tiklamayi siniflar: (ad, kind, idx)."""
    for name in reversed(WIN_ORDER):
        w = WINDOWS[name]
        if not w["open"]:
            continue
        ww, wh = WIN_DEFS[name][0], win_h(name)
        px, py = x - w["x"], y - w["y"]
        if not (0 <= px <= ww and 0 <= py <= wh):
            continue
        if py < 18:
            return (name, "close", 0) if px > ww - 20 else (name, "title", 0)
        if name == "SENARYO":
            vis_i = int((py - 24) // 22)
            ri = w.get("scroll", 0) + vis_i   # kaydirma ofseti
            if 0 <= vis_i < SCN_MAX_VIS and ri < len(get_scn_rows()):
                return (name, "row", ri)
        elif name == "AYAR":
            ri = int((py - 24) // 22)
            if 0 <= ri < len(AYAR_ROWS):
                return (name, "row", ri)
        return (name, "body", 0)
    return None


def draw_unknown_tag(frame, tag):
    corners = np.array(tag.corners, dtype=np.int32)
    cv2.polylines(frame, [corners], True, (0, 0, 255), 2)
    cv2.putText(frame, f"BILINMEYEN ID:{tag.tag_id}",
                (int(tag.center[0]) + 10, int(tag.center[1]) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)


def draw_robots(frame, robot_states):
    for rid, st in robot_states.items():
        if not st["found"]:
            continue
        rx, ry = int(st["x"]), int(st["y"])
        rangle = st["angle"]
        color, name_str = ROBOT_COLORS[rid]
        arrow_len = 25
        end_x = int(rx + arrow_len * math.cos(math.radians(rangle)))
        end_y = int(ry + arrow_len * math.sin(math.radians(rangle)))
        cv2.arrowedLine(frame, (rx, ry), (end_x, end_y), color, 2, tipLength=0.3)
        cv2.circle(frame, (rx, ry), 6, color, -1)
        if st.get("predicted"):
            cv2.circle(frame, (rx, ry), 12, (160, 160, 160), 1)  # tahmin (okluzyon)
        label = f"{name_str} ID:{rid} A:{int(rangle)}" + (" ~" if st.get("predicted") else "")
        cv2.putText(frame, label, (rx + 10, ry - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


def draw_diagnostics(frame, mission, robot_states, last_known, px_per_cm):
    """TANI KATMANI ([G] acikken — salt gozlem, sim portu): kanal biyiklari,
    'neden duruyor' rozetleri, slot/havuc/zincir, slot panosu, kavsak etiketi.
    Kontrole etkisi YOK; yalniz DBG/mission'dan okur."""
    # --- KANAL KARAR BIYIKLARI: 'neden ucgen oldu/olmadi' kaniti ---
    for (wx, wy, wnx, wny, wfL, wfR) in mission.whiskers:
        ch = (wfL < 220.0 and wfR < 220.0)
        cw = (80, 80, 235) if ch else (62, 72, 62)
        cv2.line(frame, (int(wx), int(wy)),
                 (int(wx + wnx * min(wfL, 220.0) * 0.45),
                  int(wy + wny * min(wfL, 220.0) * 0.45)), cw, 1)
        cv2.line(frame, (int(wx), int(wy)),
                 (int(wx - wnx * min(wfR, 220.0) * 0.45),
                  int(wy - wny * min(wfR, 220.0) * 0.45)), cw, 1)
    # --- BEKLEME ZINCIRI baglari R3-R1-R2 (yesil=hedef, sari=duty, kirmizi=acik) ---
    S = FORMATION_SPACING
    rel_ = (SLOT_GAP_RATIO + 0.15) * S
    stp_ = (SLOT_GAP_RATIO + 0.55) * S
    for a_, b_ in ((3, 1), (1, 2)):
        sa, sb = robot_states.get(a_), robot_states.get(b_)
        if (sa and sb and sa["found"] and sb["found"] and not mission.split
                and not mission.parked[a_] and not mission.parked[b_]
                and mission.follower_final_targets[a_][0] is None
                and mission.follower_final_targets[b_][0] is None):
            gp = math.hypot(sa["x"] - sb["x"], sa["y"] - sb["y"])
            col = ((110, 200, 90) if gp <= rel_ else
                   (70, 205, 235) if gp <= stp_ else (80, 80, 240))
            cv2.line(frame, (int(sa["x"]), int(sa["y"])),
                     (int(sb["x"]), int(sb["y"])), col, 1)
            cv2.putText(frame, "%d" % gp, (int((sa["x"] + sb["x"]) / 2),
                        int((sa["y"] + sb["y"]) / 2) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
    # --- SLOT (ici bos kare) + HAVUC (dolu nokta) + rozet, robot basina ---
    for rid in (1, 2, 3):
        st = robot_states.get(rid)
        if not st or not st["found"]:
            continue
        d = DBG[rid]
        col = ROBOT_COLORS[rid][0]
        sl = d.get("slot")
        if sl:
            cv2.rectangle(frame, (int(sl[0]) - 6, int(sl[1]) - 6),
                          (int(sl[0]) + 6, int(sl[1]) + 6), col, 1)
        ca = d.get("carrot")
        if ca:
            cv2.circle(frame, (int(ca[0]), int(ca[1])), 3, col, -1)
        ah = d.get("ahead")
        if ah and last_known.get(ah[0]) and last_known[ah[0]]["x"] is not None:
            ob = last_known[ah[0]]
            cv2.line(frame, (int(st["x"]), int(st["y"])),
                     (int(ob["x"]), int(ob["y"])), (80, 215, 250), 1)
        sttxt = d.get("state")
        if sttxt:
            cv2.putText(frame, sttxt, (int(st["x"]) - 28, int(st["y"]) + 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, state_color(sttxt), 1)
    # --- KAVSAK karar etiketi + 220px yarisma cemberi (bolunmede) ---
    if mission.split:
        for (mqx, mqy, _s) in mission.merge_pts:
            cv2.circle(frame, (int(mqx), int(mqy)), 220, (190, 190, 215), 1)
        for rid in (1, 2, 3):
            mt = DBG[rid].get("mtxt")
            mp = DBG[rid].get("merge")
            if mt and mp and str(DBG[rid].get("state", "")).startswith("KAVSAK"):
                cv2.putText(frame, mt, (int(mp[0]) - 60, int(mp[1]) - 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 90, 230), 1)
    # --- PARK SLOT PANOSU: numara + sahip rengi / BOS ---
    for si, sp in enumerate(mission.park_slots):
        owner = None
        for r_, fv in mission.follower_final_targets.items():
            if (fv[0] is not None and abs(fv[0] - sp[0]) < 2
                    and abs(fv[1] - sp[1]) < 2):
                owner = r_
                break
        cs = ROBOT_COLORS[owner][0] if owner else (120, 120, 132)
        cv2.rectangle(frame, (int(sp[0]) - 10, int(sp[1]) - 10),
                      (int(sp[0]) + 10, int(sp[1]) + 10), cs, 1)
        cv2.putText(frame, "S%d%s" % (si + 1, "" if owner else " bos"),
                    (int(sp[0]) - 10, int(sp[1]) + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, cs, 1)


def draw_windows(frame, mission, robot_states, last_known):
    """[T] baslatici serit + acik TANI pencereleri (suruklenir, salt-gozlem)."""
    F = cv2.FONT_HERSHEY_SIMPLEX
    if LAUNCHER["open"]:
        for i, n in enumerate(WIN_DEFS):
            bx = 8 + i * 78
            on = WINDOWS[n]["open"]
            cv2.rectangle(frame, (bx, LAUNCH_Y), (bx + 74, LAUNCH_Y + 18),
                          (66, 66, 86) if on else (40, 40, 50), -1)
            cv2.rectangle(frame, (bx, LAUNCH_Y), (bx + 74, LAUNCH_Y + 18),
                          (110, 110, 130), 1)
            cv2.putText(frame, n, (bx + 5, LAUNCH_Y + 13), F, 0.4,
                        (150, 230, 160) if on else (185, 185, 195), 1)
    for name in WIN_ORDER:
        if WINDOWS[name]["open"]:
            _draw_window(frame, name, mission, robot_states, last_known)


def _draw_window(frame, tab, mission, robot_states, last_known):
    F = cv2.FONT_HERSHEY_SIMPLEX
    w = WINDOWS[tab]
    px, py = w["x"], w["y"]
    pw, ph = WIN_DEFS[tab][0], win_h(tab)
    ov = frame.copy()
    cv2.rectangle(ov, (px, py), (px + pw, py + ph), (24, 24, 30), -1)
    cv2.addWeighted(ov, 0.82, frame, 0.18, 0, frame)
    cv2.rectangle(frame, (px, py), (px + pw, py + ph), (110, 110, 130), 1)
    cv2.rectangle(frame, (px, py), (px + pw, py + 18), (45, 45, 56), -1)
    cv2.putText(frame, tab, (px + 6, py + 13), F, 0.42, (210, 210, 220), 1)
    cv2.putText(frame, "x", (px + pw - 14, py + 14), F, 0.5, (120, 120, 255), 1)
    cy = [py + 34]

    def line(txt, col=(205, 205, 215)):
        cv2.putText(frame, txt, (px + 8, cy[0]), F, 0.4, col, 1)
        cy[0] += 16

    if tab == "RAPOR":
        cur, prev = REPORT["cur"], REPORT["prev"]
        if not cur:
            line("(gorev tamamlaninca otomatik dolar)")
        else:
            def c(k, fmt="%.0f", sub=None):
                cv = cur[k] if sub is None else cur[k][sub]
                pv = (prev[k] if sub is None else prev[k][sub]) if prev else None
                if pv is None:
                    return fmt % cv
                tg = "=" if abs(cv - pv) < 1e-9 else ("v" if cv < pv else "^")
                return (fmt % cv) + " (onceki " + (fmt % pv) + " %s)" % tg
            line("sure: %s sn  lider: %s sn" % (c("sure"), c("lider")))
            line("aralik R3-R1: %s px" % c("d31"))
            line("aralik R1-R2: %s px" % c("d12"))
            line("lider sapma ort/maks: %s / %s" % (c("sap_o", "%.1f"),
                                                    c("sap_m", "%.1f")))
            line("pivot%%: R3=%s R1=%s R2=%s" % (c("piv", "%d", 3),
                 c("piv", "%d", 1), c("piv", "%d", 2)))
            line("durus%%: R3=%s R1=%s R2=%s" % (c("idle", "%d", 3),
                 c("idle", "%d", 1), c("idle", "%d", 2)))
            line("v=iyilesti ^=kotulesti (dusuk iyi)", (150, 150, 160))
    elif tab == "INCELE":
        rid = SEL_ROBOT["rid"]
        st = robot_states.get(rid)
        d = DBG[rid]
        line("Robot %s (robota tikla=sec)" % ROBOT_COLORS[rid][1],
             ROBOT_COLORS[rid][0])
        line("durum: %s" % d.get("state", "-"), state_color(d.get("state", "-")))
        if st and st["found"]:
            line("konum: (%.0f,%.0f) aci %.0f" % (st["x"], st["y"], st["angle"]))
        line("hiz: L%+d R%+d" % (LAST_CMDS[rid][0], LAST_CMDS[rid][1]))
        line("hedefe: %.0f px  aci hata %+.0f" % (d.get("fdist", -1),
                                                  d.get("adiff", 0)))
        line("fuzzy: %.2f  csc: %s" % (d.get("acc", 1.0),
             "%.2f" % d["csc"] if "csc" in d else "-"))
        line("vorteks |v|=%.2f" % math.hypot(*d.get("vortex", (0, 0))))
    elif tab == "HIZ":
        line("takipci hizlari (px/s, son ~35sn)")
        gx, gy, gw, gh = px + 8, py + 38, pw - 16, ph - 46
        cv2.rectangle(frame, (gx, gy), (gx + gw, gy + gh), (50, 50, 60), 1)
        if len(SPDH) >= 2:
            for gi, rid in ((1, 1), (2, 2), (3, 3)):
                pts = [(gx + int(gw * k / (len(SPDH) - 1)),
                        gy + gh - int(min(1.0, SPDH[k][gi] / 150.0) * gh))
                       for k in range(len(SPDH))]
                for k in range(1, len(pts)):
                    cv2.line(frame, pts[k - 1], pts[k], ROBOT_COLORS[rid][0], 1)
    elif tab == "PWM":
        line("teker komutlari (L / R)")
        for rid in (0, 3, 1, 2):
            cv2.putText(frame, ROBOT_COLORS[rid][1], (px + 8, cy[0] + 4), F,
                        0.4, ROBOT_COLORS[rid][0], 1)
            for side in (0, 1):
                v = LAST_CMDS[rid][side]
                bx0 = px + 60 + side * 118
                mid = bx0 + 48
                cv2.rectangle(frame, (bx0, cy[0] - 6), (bx0 + 96, cy[0] + 5),
                              (52, 52, 62), 1)
                wpx = int(48 * max(-1.0, min(1.0, v / 50.0)))
                cc = (115, 200, 95) if v >= 0 else (70, 110, 240)
                cv2.rectangle(frame, (min(mid, mid + wpx), cy[0] - 5),
                              (max(mid, mid + wpx), cy[0] + 4), cc, -1)
            cy[0] += 22
    elif tab == "FUZZY":
        rid = SEL_ROBOT["rid"]
        d = DBG[rid]
        gx, gy, gw, gh = px + 10, py + 30, pw - 20, 96
        cv2.rectangle(frame, (gx, gy), (gx + gw, gy + gh), (50, 50, 60), 1)
        prof = FUZZY_PROFILES["line"]

        def rx(rt):
            return gx + int(gw * min(2.5, rt) / 2.5)

        def ry(mu):
            return gy + gh - int(mu * (gh - 5)) - 3
        dn, ct, sf = prof["danger"], prof["caution"], prof["safe"]
        cv2.polylines(frame, [np.array([(rx(dn[0]), ry(1)), (rx(dn[2]), ry(1)),
                      (rx(dn[3]), ry(0))], np.int32)], False, (70, 70, 240), 1)
        cv2.polylines(frame, [np.array([(rx(ct[0]), ry(0)), (rx(ct[1]), ry(1)),
                      (rx(ct[2]), ry(0))], np.int32)], False, (80, 215, 250), 1)
        cv2.polylines(frame, [np.array([(rx(sf[0]), ry(0)), (rx(sf[1]), ry(1)),
                      (rx(2.5), ry(1))], np.int32)], False, (110, 215, 120), 1)
        cy[0] = gy + gh + 14
        ah = d.get("ahead")
        if ah:
            rt = ah[1] / FORMATION_SPACING
            cv2.line(frame, (rx(rt), gy), (rx(rt), gy + gh),
                     ROBOT_COLORS[rid][0], 1)
            line("%s: oran=%.2f onum=R%d cikti=%.2f" % (ROBOT_COLORS[rid][1],
                 rt, ah[0], d.get("acc", 1.0)), ROBOT_COLORS[rid][0])
        else:
            line("%s: onunde robot yok (cikti=1.0)" % ROBOT_COLORS[rid][1])
        line("kirmizi=danger sari=caution yesil=safe", (150, 150, 160))
    elif tab == "SENARYO":
        rows_all = get_scn_rows()
        scroll = max(0, min(w.get("scroll", 0), max(0, len(rows_all) - SCN_MAX_VIS)))
        w["scroll"] = scroll   # clamp (dosya silinince tasma olmasin)
        for vis_i in range(min(SCN_MAX_VIS, len(rows_all) - scroll)):
            kind, nm, _r = rows_all[scroll + vis_i]
            ry0 = py + 24 + vis_i * 22
            cc = (60, 80, 60) if kind == "save" else (40, 48, 60)
            cv2.rectangle(frame, (px + 6, ry0), (px + pw - 6, ry0 + 19), cc, -1)
            cv2.putText(frame, nm[:42], (px + 10, ry0 + 14), F, 0.38,
                        (205, 215, 205), 1)
            if kind == "file":
                cv2.rectangle(frame, (px + pw - 46, ry0), (px + pw - 6, ry0 + 19),
                              (40, 40, 90), -1)
                cv2.putText(frame, "SIL", (px + pw - 40, ry0 + 14), F, 0.38,
                            (150, 150, 255), 1)
        # kaydirma ipucu (tekerlekle gezin)
        if len(rows_all) > SCN_MAX_VIS:
            hint = "tekerlek: %d-%d / %d" % (scroll + 1,
                   min(scroll + SCN_MAX_VIS, len(rows_all)), len(rows_all))
            cv2.putText(frame, hint, (px + pw - 130, py + 13), F, 0.34,
                        (150, 200, 150), 1)
    elif tab == "AYAR":
        # NOT: satirlar py+24'ten baslar (windows_hit ile birebir); BASLIK SATIRI
        # YOK — eskiden 'tikla->uygula' yazisi row-0'in ustune binip okunmuyordu.
        for i, (nm, pv) in enumerate(AYAR_ROWS):
            act = abs(COLLISION_CONTACT_PAD - pv) < 0.1
            ry0 = py + 24 + i * 22
            cv2.rectangle(frame, (px + 6, ry0), (px + pw - 6, ry0 + 19),
                          (62, 62, 80) if act else (44, 44, 54), -1)
            cv2.putText(frame, ("> " if act else "  ") + nm, (px + 10, ry0 + 14),
                        F, 0.4, (150, 230, 150) if act else (205, 205, 215), 1)
        cv2.putText(frame, "60px gercekte sinir (37 sim-ozel alinmadi)",
                    (px + 8, py + 24 + len(AYAR_ROWS) * 22 + 12), F, 0.34,
                    (150, 140, 110), 1)


def draw_ruler(frame, px_per_cm):
    """Orta-tik cetvel: px + (olcek varsa) cm."""
    if RULER["a"] and RULER["b"]:
        a, b = RULER["a"], RULER["b"]
        cv2.line(frame, a, b, (255, 220, 120), 1)
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        txt = "%.0f px" % d
        if px_per_cm:
            txt += " = %.1f cm" % (d / px_per_cm)
        cv2.putText(frame, txt, (int((a[0] + b[0]) / 2) + 8,
                    int((a[1] + b[1]) / 2) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 220, 120), 1)


def draw_walls(frame, mission):
    """Dikdortgen engeller (yari saydam dolgu + parlak kenar) + GRID editor katmani."""
    if mission.walls:
        ov = frame.copy()
        for (rx1, ry1, rx2, ry2) in mission.walls:
            cv2.rectangle(ov, (int(rx1), int(ry1)), (int(rx2), int(ry2)), (0, 0, 200), -1)
        cv2.addWeighted(ov, 0.30, frame, 0.70, 0, frame)
        for (rx1, ry1, rx2, ry2) in mission.walls:
            cv2.rectangle(frame, (int(rx1), int(ry1)), (int(rx2), int(ry2)), (0, 0, 230), 2)
    if mission.wall_mode:
        # Minecraft-tarzi grid (sim portu): hucre cizgileri + uzerinde durulan hucre
        fh, fw = frame.shape[:2]
        for gx in range(0, fw + 1, GRID):
            cv2.line(frame, (gx, 0), (gx, fh), (90, 90, 90), 1)
        for gy in range(0, fh + 1, GRID):
            cv2.line(frame, (0, gy), (fw, gy), (90, 90, 90), 1)
        if mission.wall_preview is not None:
            hx, hy = mission.wall_preview
            cx, cy = (hx // GRID) * GRID, (hy // GRID) * GRID
            cv2.rectangle(frame, (cx, cy), (cx + GRID, cy + GRID), (255, 255, 255), 1)
        cv2.putText(frame, "[E] DUVAR MODU  Sol tik/surukle=hucre boya  "
                           "Sag tik/surukle=sil  [C]=Temizle  (otomatik kayit)",
                    (10, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 80, 255), 2)


def draw_split_routes(frame, mission):
    """BOLUNMELI ucgen katmani: yan robotlarin OZ rotalari + kavsak noktalari."""
    if not mission.split:
        return
    for rid in mission.own_route:   # YAN robotlar (dinamik: orta haric ikisi)
        rt = mission.own_route.get(rid)
        if rt and len(rt) >= 2:
            col = ROBOT_COLORS[rid][0]
            pts = np.array([[int(q[0]), int(q[1])] for q in rt],
                           np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], False, col, 1)
    for (mqx, mqy, _s) in mission.merge_pts:
        cv2.circle(frame, (int(mqx), int(mqy)), 6, (255, 255, 255), 1)


def draw_planned_route(frame, mission, leader_lk):
    """Mavi noktali planlanan rota + 'rota hesaplaniyor' overlay'i."""
    if not mission.planned_wps:
        return
    sx = int(leader_lk["x"]) if leader_lk["x"] is not None else 0
    sy = int(leader_lk["y"]) if leader_lk["y"] is not None else 0
    pts = [(sx, sy)] + [(int(p[0]), int(p[1])) for p in mission.planned_wps]
    for i in range(len(pts) - 1):
        cv2.line(frame, pts[i], pts[i + 1], (255, 180, 0), 1)
        cv2.circle(frame, pts[i + 1], 4, (255, 180, 0), -1)
    if time.time() - mission.route_calc_t < 2.0:
        cv2.putText(frame, "ROTA HESAPLANIYOR...", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)


def draw_target_and_trace(frame, mission):
    if mission.target_x is not None:
        hx, hy = mission.target_x, mission.target_y
        cv2.drawMarker(frame, (hx, hy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.circle(frame, (hx, hy), ARRIVAL_RADIUS, (0, 100, 255), 1)
    if len(mission.leader_path) > 1:
        pts = np.array([[int(p[0]), int(p[1])] for p in mission.leader_path],
                       np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, [pts], False, (0, 165, 255), 1)


def draw_follower_target(frame, rid, fx, fy, tx, ty):
    c = FOLLOWER_TARGET_COLORS[rid]
    cv2.drawMarker(frame, (int(tx), int(ty)), c, cv2.MARKER_TILTED_CROSS, 10, 1)
    cv2.line(frame, (int(fx), int(fy)), (int(tx), int(ty)), c, 1, cv2.LINE_AA)


def draw_hud(frame, status_text, status_color, robot_pwms, speed_scale,
             px_per_cm, fps_display, serial_status, camera_warn=None,
             tag_stats=""):
    fh, fw = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (fw, 76), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    cv2.putText(frame, status_text, (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2)
    cv2.putText(frame, f"L: L:{robot_pwms[0][0]:+d} R:{robot_pwms[0][1]:+d} | "
                       f"T1: L:{robot_pwms[1][0]:+d} R:{robot_pwms[1][1]:+d}",
                (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
    cv2.putText(frame, f"T2: L:{robot_pwms[2][0]:+d} R:{robot_pwms[2][1]:+d} | "
                       f"T3: L:{robot_pwms[3][0]:+d} R:{robot_pwms[3][1]:+d}",
                (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

    scale_str = f"{FORMATION_SPACING / px_per_cm:.0f}cm" if px_per_cm else "?"
    cv2.putText(frame, f"Hiz: x{speed_scale:.1f}  Aralik: {int(FORMATION_SPACING)}px "
                       f"(~{scale_str})  FPS:{fps_display}  {serial_status}",
                (fw - 380, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

    if camera_warn:
        cv2.putText(frame, camera_warn, (fw - 380, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

    if tag_stats:
        cv2.putText(frame, tag_stats, (fw - 380, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (170, 170, 170), 1)

    shortcuts = ("[Sol] Hedef  [Sag] Temizle  [1]Cizgi [2]Ucgen  [S]Dur  "
                 "[+/-]Hiz  [E]Duvar [C]Temizle  [G]Tani [orta-tik]Cetvel  [Q]Cikis")
    cv2.putText(frame, shortcuts, (10, fh - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)


def draw_safety_banner(frame, text):
    """Kamera watchdog tetiklenince tam genislik kirmizi banner."""
    fh, fw = frame.shape[:2]
    ov = frame.copy()
    cv2.rectangle(ov, (0, fh // 2 - 40), (fw, fh // 2 + 40), (0, 0, 180), -1)
    cv2.addWeighted(ov, 0.75, frame, 0.25, 0, frame)
    cv2.putText(frame, text, (max(10, fw // 2 - 330), fh // 2 + 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)


def make_mouse_callback(mission, get_leader_start, plan_fn, on_new_target=None,
                        get_follower_pos=None, get_robot_pos=None):
    """Fare callback'i: TANI pencereleri + GRID duvar modu (E) + hedef sec/temizle.
    get_leader_start() -> (x, y) veya (None, None); plan_fn = plan_path;
    get_follower_pos() -> {rid: (x,y)} (bolunmede en-yakin yan atamasi icin);
    get_robot_pos() -> {rid: (x,y)} (INCELE penceresinde robot secimi)."""

    def _cell_edit(x, y, add):
        cell = (x // GRID, y // GRID)
        if add:
            mission.grid_cells.add(cell)
        else:
            mission.grid_cells.discard(cell)
        mission.walls[:] = merged_walls(mission.grid_cells)
        save_parkur(mission.grid_cells)

    def cb(event, x, y, flags, param):
        global COLLISION_CONTACT_PAD
        # --- TANI PENCERELERI (en ust oncelik — sahaya gecmez) ---
        # SENARYO uzerinde tekerlek = listede kaydir
        if event == getattr(cv2, "EVENT_MOUSEWHEEL", -999):
            over = windows_hit(x, y)
            if over and over[0] == "SENARYO":
                delta = cv2.getMouseWheelDelta(flags) if hasattr(
                    cv2, "getMouseWheelDelta") else flags
                w = WINDOWS["SENARYO"]
                step = -1 if delta > 0 else 1
                w["scroll"] = max(0, min(w.get("scroll", 0) + step,
                                         scn_scroll_max()))
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            whit = windows_hit(x, y)
            if whit:
                wname, kind, idx = whit
                WIN_ORDER.remove(wname); WIN_ORDER.append(wname)   # one getir
                if kind == "close":
                    WINDOWS[wname]["open"] = False
                elif kind == "title":
                    WINDOWS[wname]["drag"] = (x - WINDOWS[wname]["x"],
                                              y - WINDOWS[wname]["y"])
                elif kind == "row" and wname == "AYAR":
                    COLLISION_CONTACT_PAD = AYAR_ROWS[idx][1]
                    print("[AYAR] %s (temas=%.0fpx)" % (AYAR_ROWS[idx][0],
                          52 + COLLISION_CONTACT_PAD))
                elif kind == "row" and wname == "SENARYO":
                    skind, snm, sref = get_scn_rows()[idx]
                    if skind == "save":
                        fn = save_named_parkur(mission.grid_cells)
                        _SCN_CACHE["t"] = 0.0   # listeyi zorla tazele
                        print("[PARKUR] kaydedildi: %s" % (os.path.basename(fn)
                              if fn else "cizim yok/HATA"))
                    elif skind == "file" and x - WINDOWS["SENARYO"]["x"] > \
                            WIN_DEFS["SENARYO"][0] - 46:
                        try:
                            os.remove(sref); _SCN_CACHE["t"] = 0.0
                            print("[PARKUR] silindi: %s" % snm)
                        except OSError:
                            print("[PARKUR] silinemedi")
                    elif skind == "file":
                        try:
                            mission.grid_cells.clear()
                            mission.grid_cells.update(
                                tuple(c) for c in json.load(
                                    open(sref, encoding="utf-8")))
                            mission.walls[:] = merged_walls(mission.grid_cells)
                            save_parkur(mission.grid_cells)
                            mission.clear_target()
                            print("[PARKUR] yuklendi: %s" % snm)
                        except (OSError, ValueError):
                            print("[PARKUR] okunamadi")
                return
            lname = launcher_hit(x, y)
            if lname:
                WINDOWS[lname]["open"] = not WINDOWS[lname]["open"]
                if WINDOWS[lname]["open"]:
                    WIN_ORDER.remove(lname); WIN_ORDER.append(lname)
                return
            # INCELE acikken robota tik = sec (hedef koymaz)
            if WINDOWS["INCELE"]["open"] and get_robot_pos:
                for rid, p in get_robot_pos().items():
                    if p[0] is not None and math.hypot(p[0] - x, p[1] - y) < 26:
                        SEL_ROBOT["rid"] = rid
                        return
        elif event == cv2.EVENT_LBUTTONUP:
            for w in WINDOWS.values():
                w["drag"] = None
        elif event == cv2.EVENT_MOUSEMOVE and any(
                w["drag"] for w in WINDOWS.values()):
            for w in WINDOWS.values():
                if w["drag"]:
                    w["x"] = max(0, min(CAM_WIDTH - 80, x - w["drag"][0]))
                    w["y"] = max(0, min(CAM_HEIGHT - 40, y - w["drag"][1]))
            return

        # --- DUVAR MODU (sim portu): Minecraft-tarzi hucre boyama ---
        # Sol tik/surukle = hucre ekle, sag tik/surukle = hucre sil.
        if mission.wall_mode:
            if event == cv2.EVENT_MOUSEMOVE:
                mission.wall_preview = (x, y)   # uzerinde durulan hucre vurgusu
                if flags & cv2.EVENT_FLAG_LBUTTON:
                    _cell_edit(x, y, True)
                elif flags & cv2.EVENT_FLAG_RBUTTON:
                    _cell_edit(x, y, False)
            elif event == cv2.EVENT_LBUTTONDOWN:
                _cell_edit(x, y, True)
            elif event == cv2.EVENT_RBUTTONDOWN:
                _cell_edit(x, y, False)
            return   # duvar modunda hedef tiklama devre disi

        # ORTA-TIK CETVEL (sim portu): iki nokta arasi px + cm olcumu
        if event == cv2.EVENT_MBUTTONDOWN:
            RULER["a"] = (x, y); RULER["b"] = (x, y); RULER["live"] = True
            return
        elif event == cv2.EVENT_MOUSEMOVE and RULER["live"]:
            RULER["b"] = (x, y)
            return
        elif event == cv2.EVENT_MBUTTONUP:
            RULER["live"] = False
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            fpos = get_follower_pos() if get_follower_pos else None
            mission.set_target(x, y, get_leader_start(), plan_fn, fpos=fpos)
            if on_new_target:
                on_new_target()
        elif event == cv2.EVENT_RBUTTONDOWN:
            mission.clear_target()

    return cb


def print_banner():
    print("\n" + "=" * 60)
    print("  LIDER & TAKIPCI ROBOT NAVIGASYON SISTEMI HAZIR")
    print("  Sol tik : Hedef belirle")
    print("  Sag tik : Hedef temizle")
    print("  [1] Cizgi Formasyonu  [2] Ucgen Formasyonu")
    print("  [E] Duvar modu (GRID: sol tik/surukle=hucre boya, sag=sil; otomatik kayit)  [C] Temizle")
    print("  [S] Dur/Devam  [+]/[-] Hiz ayari  [o]/[p] Aralik  [Q] Cikis")
    print("=" * 60 + "\n")

# ===================== ANA DONGU =====================
WINDOW_NAME = "Lider Robot Navigasyon"


def main():
    parse_args()

    # Windows DPI olceklemesi: fare koordinatlarinin pikselle eslesmesi icin
    try:
        ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

    mission = Mission()
    paused = False
    speed_scale = 0.67
    formation_mode = "line"   # LINE ile basla — engelden ip gibi gececekler
    analyze = False           # [G] tani katmani (varsayilan KAPALI — temiz kamera)

    trackers = {rid: RobotTracker() for rid in ROBOT_IDS}

    # --- Seri TX thread (write_timeout + deadman + reconnect, bkz. SERI HABERLESME bolumu) ---
    writer = SerialWriter()
    writer.start()

    # --- Kamera ---
    camera = LatencyFreeCamera()
    if not camera.running:
        print("[HATA] Kamera baslatilamadi. Cikiliyor.")
        writer.stop()
        return

    # CPU bol -> quad_decimate=1.0 (TAM cozunurluk) en isabetli konum/aci -> jitter azalir.
    # quad_sigma kucuk blur ile kose tespiti kararli kalir. nthreads makineye gore.
    detector = Detector(
        families=APRILTAG_FAMILIES,
        nthreads=os.cpu_count() or 4,
        quad_decimate=1.0,
        quad_sigma=0.5,
    )

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    # Pencereyi kare boyutuna sabitle: 1:1 eslesme -> fare koordinati guvenilir.
    # NOT: Pencereyi elle buyutursen bir kez test et (robota tikla, isaret ustune
    # dusuyor mu?). OpenCV 4.5+ surumlerinde resize sonrasi fare koordinat
    # tutarsizliklari raporlanmistir; sorun gorursen pencereyi bu boyutta kullan.
    cv2.resizeWindow(WINDOW_NAME, CAM_WIDTH, CAM_HEIGHT)

    def get_leader_start():
        snap = trackers[LEADER_ID].snapshot()
        return (snap["x"], snap["y"])

    def plan_in_frame(start, goal, walls):
        # Rota dugumleri kare disina tasamasin (kamera oradan takip edemez)
        return plan_path(start, goal, walls,
                         bounds=(CAM_WIDTH, CAM_HEIGHT))

    def get_follower_pos():
        # Bolunmede DINAMIK orta + 'en yakin yan' atamasi icin TUM takipci konumlari
        # (R1 dahil -> rota merkezine en yakin olan ortadan gecer). Gorunmuyorsa None.
        out = {}
        for rid in FOLLOWER_IDS:
            s = trackers[rid].snapshot()
            out[rid] = (s["x"], s["y"])
        return out

    def get_robot_pos():
        # INCELE penceresinde robot secimi (1,2,3)
        return {rid: (trackers[rid].snapshot()["x"], trackers[rid].snapshot()["y"])
                for rid in FOLLOWER_IDS}

    cv2.setMouseCallback(WINDOW_NAME, make_mouse_callback(
        mission, get_leader_start, plan_in_frame,
        on_new_target=reset_pd,   # eski hedefin PD turev 'hayaletini' temizle
        get_follower_pos=get_follower_pos,
        get_robot_pos=get_robot_pos,
    ))

    # Yumusak formasyon gecisi: her takipcinin slot degerleri hedefe kademeli kayar
    current_slots = {rid: dict(FORMATIONS[formation_mode][rid]) for rid in FOLLOWER_IDS}

    undistort_maps = (None, None)   # ilk karede frame boyutu bilinince kurulur
    undistort_ready = False
    px_per_cm = None                # tag kenarlarindan canli olculur (medyan + EMA)
    resize_warned = False

    last_seq = -1
    last_frame_t = None
    last_rendered = None            # stale ekraninda gosterilecek son cizim
    frame_count = 0
    fps_time = time.time()
    fps_display = 0
    last_log_time = 0.0

    # Tag-kayip sayaci (UCUZ: kare basina 4 toplama; metin 5 sn'de BIR kurulur)
    tag_miss = {rid: 0 for rid in ROBOT_IDS}
    tag_win_frames = 0
    tag_win_start = time.time()
    tag_stats_text = ""

    log_name = (time.strftime("nav_log_%Y%m%d_%H%M%S.txt")
                if LOG_TIMESTAMPED else "nav_log.txt")
    _log_file = open(log_name, "w", encoding="utf-8")
    # Eski kolonlar AYNEN korunur (analyze_log.py uyumu); PWM kolonlari SONA eklendi.
    _log_file.write("time,id0x,id0y,id0a,id1x,id1y,id1a,id2x,id2y,id2a,id3x,id3y,id3a,"
                    "d02,d01,d03,d12,d13,d23,act2,act1,act3,ldr_dist,form,park1,park3,park2,status,"
                    "pwm0l,pwm0r,pwm1l,pwm1r,pwm2l,pwm2r,pwm3l,pwm3r,st1,st2,st3,"
                    # --- B-LISTE (MATLAB figurleri icin) ---
                    "v0,v1,v2,v3,acc1,acc2,acc3,dah1,dah2,dah3,csc1,csc2,csc3,"
                    "tx1,ty1,tx2,ty2,tx3,ty3,ldev,split,pxpercm,fps,"
                    "lead1,lead2,lead3\n")   # lead*: UCGEN cohesion girdisi (COM-offset norm)
    _log_file.flush()
    print(f"[LOG] {log_name}")

    print_banner()
    auto_exit_t = None   # 3/3 park aninda set edilir -> kisa sure sonra oto-cikis

    # ------------------------------------------------------------------
    def handle_keys():
        """Ortak tus isleyici. True donerse cikis istendi."""
        nonlocal paused, formation_mode, speed_scale, analyze
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            return True
        elif key == ord("g") or key == ord("G"):
            analyze = not analyze
            print(f"[ANALIZ] Tani katmani {'ACIK' if analyze else 'kapali'}")
        elif key == ord("t") or key == ord("T"):
            LAUNCHER["open"] = not LAUNCHER["open"]
            print(f"[PANEL] Pencere baslatici {'ACIK' if LAUNCHER['open'] else 'kapali'}")
        elif key == ord("s") or key == ord("S"):
            paused = not paused
            print(f"[{'DURDURULDU' if paused else 'DEVAM'}]")
        elif key == ord("1"):
            formation_mode = "line"
            print("[FORMASYON] Cizgi (Line) moduna gecildi.")
        elif key == ord("2"):
            formation_mode = "triangle"
            print("[FORMASYON] Ucgen (Triangle) moduna gecildi.")
        elif key == ord("+") or key == ord("="):
            speed_scale = min(2.0, speed_scale + 0.1)
            print(f"[HIZ] x{speed_scale:.1f}")
        elif key == ord("-") or key == ord("_"):
            speed_scale = max(0.3, speed_scale - 0.1)
            print(f"[HIZ] x{speed_scale:.1f}")
        elif key == ord("e") or key == ord("E"):
            mission.wall_mode = not mission.wall_mode
            mission.wall_pending = None
            print(f"[DUVAR] Mod: "
                  f"{'ACIK (GRID)  Sol tik/surukle=hucre boya  Sag tik/surukle=sil' if mission.wall_mode else 'KAPALI'}")
        elif key == ord("c") or key == ord("C"):
            mission.clear_walls_and_route()
        elif key == ord("o") or key == ord("O"):
            set_formation_spacing(FORMATION_SPACING - 10.0)
            print(f"[FORMASYON ARALIGI] {FORMATION_SPACING}px")
        elif key == ord("p") or key == ord("P"):
            set_formation_spacing(FORMATION_SPACING + 10.0)
            print(f"[FORMASYON ARALIGI] {FORMATION_SPACING}px")
        return False

    # ------------------------------------------------------------------
    try:
        while True:
            seq, frame, t_frame = camera.read()
            now = time.time()

            # ====== 1) KAMERA WATCHDOG: kare yok veya BAYAT -> guvenli durdur ======
            if frame is None or (now - t_frame) > FRAME_STALE_TIMEOUT:
                writer.set_all_zero()
                canvas = (last_rendered.copy() if last_rendered is not None
                          else np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), np.uint8))
                draw_safety_banner(canvas, "KAMERA SINYALI YOK - ROBOTLAR DURDURULDU")
                cv2.imshow(WINDOW_NAME, canvas)
                if handle_keys():
                    break
                time.sleep(0.02)
                continue

            # ====== 2) KARE DEDUP: ayni kareyi tekrar isleme ======
            # (CPU ~%40-50 tasarruf + hiz tahmininin sifira dogru cekilme bias'i kalkar.
            #  Robotlar son PWM'i SerialWriter cadence'iyle almaya devam eder.)
            if seq == last_seq:
                if handle_keys():
                    break
                time.sleep(0.004)
                continue
            last_seq = seq

            if last_frame_t is None:
                dt = 1.0 / max(1, CAM_FPS)
            else:
                dt = max(0.0, min(0.1, t_frame - last_frame_t))   # buyuk sicramalari kis
            last_frame_t = t_frame

            # ====== 3) On isleme ======
            fh0, fw0 = frame.shape[:2]
            if (fw0, fh0) != (CAM_WIDTH, CAM_HEIGHT):
                if not resize_warned:
                    resize_warned = True
                    if abs((fw0 / fh0) - (CAM_WIDTH / CAM_HEIGHT)) > 0.01:
                        print(f"[UYARI] Kaynak {fw0}x{fh0}, hedef "
                              f"{CAM_WIDTH}x{CAM_HEIGHT}: en-boy orani farkli, "
                              f"geometri carpilir! Kaynak cozunurlugunu esitle.")
                frame = cv2.resize(frame, (CAM_WIDTH, CAM_HEIGHT),
                                   interpolation=cv2.INTER_LINEAR)
            # (kaynak zaten 1280x720 ise resize atlanir -> kare basina bedava kopya yok)

            fh, fw = frame.shape[:2]
            if not undistort_ready:
                undistort_maps = load_undistort_maps(CALIB_FILE, fw, fh)
                undistort_ready = True
            if undistort_maps[0] is not None:
                frame = cv2.remap(frame, undistort_maps[0], undistort_maps[1], cv2.INTER_LINEAR)

            frame_count += 1
            if now - fps_time >= 1.0:
                fps_display = frame_count   # islenen GERCEK kare sayisi (dup'lar haric)
                frame_count = 0
                fps_time = now

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray)

            # ====== 4) Algilama -> takip ======
            robot_states = {
                rid: {"x": None, "y": None, "angle": None, "found": False, "predicted": False}
                for rid in ROBOT_IDS
            }
            scale_samples = []
            updated = set()

            for tag in tags:
                tid = tag.tag_id
                if tid not in trackers:
                    draw_unknown_tag(frame, tag)   # listede olmayan tag'i kirmizi goster
                    continue
                if tag.decision_margin < MIN_DECISION_MARGIN:
                    continue   # dusuk guvenli tespiti reddet
                raw_x, raw_y = float(tag.center[0]), float(tag.center[1])
                corners = tag.corners
                bx = (corners[0][0] + corners[1][0]) / 2.0
                by = (corners[0][1] + corners[1][1]) / 2.0
                tx_ = (corners[3][0] + corners[2][0]) / 2.0
                ty_ = (corners[3][1] + corners[2][1]) / 2.0
                raw_angle = (math.degrees(math.atan2(ty_ - by, tx_ - bx)) + 180) % 360
                cal = ROBOT_CALIBRATION.get(tid, {})
                calibrated_angle = (raw_angle + cal.get("angle_offset", 0.0)) % 360

                # KF: ziplama reddi (innovation gate) + uzun-kayip sonrasi reinit icinde
                if not trackers[tid].update(raw_x, raw_y, calibrated_angle, t_frame):
                    continue   # sahte tespit: bu kare reddedildi, onceki konum korunur
                updated.add(tid)

                # Canli px<->cm olcek ornegi (4 kenar ortalamasi / fiziksel boyut)
                if TAG_SIZE_CM > 0:
                    edge_px = 0.0
                    for i in range(4):
                        p, q = corners[i], corners[(i + 1) % 4]
                        edge_px += math.hypot(q[0] - p[0], q[1] - p[1])
                    scale_samples.append((edge_px / 4.0) / TAG_SIZE_CM)

            # Olcek: kare icindeki TUM tag'lerin MEDYANI -> sonra yavas EMA
            # (eski kod her tag'i sirayla tek EMA'ya karistiriyordu; medyan gurultuye dayanikli)
            if scale_samples:
                m = median(scale_samples)
                px_per_cm = m if px_per_cm is None else 0.1 * m + 0.9 * px_per_cm

            # Okluzyon yonetimi: bulunamayan robotlari kisa sure KF tahminiyle surdur
            for rid in ROBOT_IDS:
                est = trackers[rid].estimate(t_frame)
                if est is None:
                    continue   # cok uzun kayip -> robot durur (found=False kalir)
                robot_states[rid]["x"] = est["x"]
                robot_states[rid]["y"] = est["y"]
                robot_states[rid]["angle"] = est["angle"]
                robot_states[rid]["found"] = True
                robot_states[rid]["predicted"] = (rid not in updated)

            last_known = {rid: trackers[rid].snapshot() for rid in ROBOT_IDS}

            # Tag-kayip sayaci: kare basina 4 artim; metin 5sn'de bir kurulur (yuk ~0)
            tag_win_frames += 1
            for rid in ROBOT_IDS:
                if rid not in updated:
                    tag_miss[rid] += 1
            if now - tag_win_start >= 5.0:
                tag_stats_text = "TAG kayip  " + "  ".join(
                    f"{ROBOT_COLORS[rid][1]}:%{100 * tag_miss[rid] // max(1, tag_win_frames)}"
                    for rid in ROBOT_IDS)
                tag_miss = {rid: 0 for rid in ROBOT_IDS}
                tag_win_frames = 0
                tag_win_start = now

            # ====== 5) Lider izini kaydet (mavi cizgiye projekte) ======
            if (not paused and robot_states[0]["found"]
                    and not robot_states[0].get("predicted") and mission.target_active):
                lx = robot_states[0]["x"]
                ly = robot_states[0]["y"]
                langle = robot_states[0]["angle"]
                lp = mission.leader_path
                if (len(lp) == 0 or
                        math.hypot(lx - lp[-1][0], ly - lp[-1][1]) > PATH_RECORD_DIST):
                    if lp:
                        mission.leader_dist_since_target += math.hypot(
                            lx - lp[-1][0], ly - lp[-1][1])
                    lp.append((lx, ly, langle))
                    # Takipciler MAVI CIZGI (planlanan rota) uzerinde gitsin:
                    # lider konumunu cizgiye projekte et.
                    if mission.planned_route:
                        (px_, py_), ph_ = project_to_polyline((lx, ly), mission.planned_route)
                    else:
                        px_, py_, ph_ = lx, ly, langle
                    mission.follow_trace.append((px_, py_, ph_))

            # ====== 6) Cizimler ======
            draw_robots(frame, robot_states)
            draw_walls(frame, mission)
            draw_planned_route(frame, mission, last_known[0])
            draw_split_routes(frame, mission)
            draw_target_and_trace(frame, mission)

            # ====== 7) LIDER kontrol ======
            robot_pwms = {rid: (0, 0) for rid in ROBOT_IDS}
            left_cmd, right_cmd = 0.0, 0.0
            status_text = "BEKLENIYOR"
            status_color = (150, 150, 150)

            if paused:
                status_text = "DURDURULDU [S]"
                status_color = (0, 100, 255)
            elif not robot_states[0]["found"]:
                status_text = "LIDER BULUNAMADI"
                status_color = (0, 0, 255)
            elif mission.target_x is None:
                if len(mission.follow_trace) > 0:
                    n_parked = sum(1 for r in FOLLOWER_IDS if mission.parked[r])
                    status_text = (f"FINISTE | Takipciler diziliyor/park ediyor... "
                                   f"({n_parked}/3 park)")
                    status_color = (0, 200, 255)
                else:
                    status_text = "HEDEF YOK (tiklayin)"
                    status_color = (200, 200, 0)
            else:
                lx = robot_states[0]["x"]
                ly = robot_states[0]["y"]
                langle = robot_states[0]["angle"]

                # --- PURE-PURSUIT: rotada ILERIDE bir 'carrot'a kilitlen ---
                # Anlik waypoint'e aim etmek yerine (waypoint YAKININCA bearing
                # hassaslasip kamera gecikmesiyle YERINDE PIVOT-kilidi yapiyordu —
                # 141241 logu: WP7'de jit 180 takilip VARAMADI) rotada ~LEADER_LOOKAHEAD
                # ileride bir nokta hedefle. Carrot ileride -> bearing KARARLI ->
                # KAVISLI yumusak donus, pivot yok. Rota zaten guvenli-marjli
                # (smooth_path) -> lider engele YAKLASMADAN gider (kullanici istegi).
                wps = mission.planned_wps
                route = mission.planned_route
                dist_final = math.hypot(mission.target_x - lx, mission.target_y - ly)
                cr = (carrot_on_path(lx, ly, route, LEADER_LOOKAHEAD)
                      if route and len(route) >= 2 else None)
                if cr is not None:
                    wp_tx, wp_ty = cr[0], cr[1]
                else:
                    wp_tx, wp_ty = mission.target_x, mission.target_y
                dist = math.hypot(wp_tx - lx, wp_ty - ly)
                # current_wp SADECE durum gostergesi icin ilerler (kontrol carrot'ta)
                if (wps and mission.current_wp < len(wps) - 1 and
                        math.hypot(wps[mission.current_wp][0] - lx,
                                   wps[mission.current_wp][1] - ly) <= LEADER_WP_RADIUS):
                    mission.current_wp += 1

                if dist_final > ARRIVAL_RADIUS:
                    # APF duvar itmesi LIDERDE DE (takipcide zaten vardi): duvara
                    # surTuklenmeden kenardan kavis cizer. Eksikti -> lider duvar
                    # kosesine girip kilitleniyordu (17:56 testi, t=19s donma).
                    wdx, wdy = wp_tx - lx, wp_ty - ly
                    wmag = math.hypot(wdx, wdy) or 1.0
                    wdx, wdy = wdx / wmag, wdy / wmag
                    rep_x, rep_y = wall_repulsion(lx, ly, mission.walls,
                                                  influence=LEADER_WALL_INFLUENCE,
                                                  gain=LEADER_WALL_GAIN)
                    rmag = math.hypot(rep_x, rep_y)
                    # ITME OLU-BANDI (yumusak): merkezdeki kucuk-net flip'ini keser
                    # -> dar gecitte zigzag yerine planli merkez cizgide DUZ gider.
                    if rmag > 1e-6:
                        _db = max(0.0, rmag - LEADER_WALL_DEADBAND) / rmag
                        rep_x *= _db
                        rep_y *= _db
                        rmag *= _db
                    # APF BARIYER ONLEYICI: dar gecit AGZINDA iki kose itmesinin toplami
                    # hedef vektorunu GERI cevirebiliyor (lider kapida doner durur).
                    # Itme buyuklugu hedef vektorunun 0.75'iyle sinirlanir: EGEBILIR,
                    # asla geri ceviremez -> lider dar kapidan da girer.
                    if rmag > 0.75:
                        rep_x *= 0.75 / rmag
                        rep_y *= 0.75 / rmag
                    # Itmenin HEDEFE-ZIT (geri) bileseni atilir: kapi agzinda kose
                    # itmesi geri itip robotu 'don-engellendi' tuzagina sokuyordu.
                    # Kalan bilesen YANAL kilavuz olur (kapidan iceri suzdurur).
                    _dg = rep_x * wdx + rep_y * wdy
                    if _dg < 0:
                        rep_x -= _dg * wdx
                        rep_y -= _dg * wdy
                    # SON YAKLASMA itme kismasi: hedef bir engele YAKIN konduysa
                    # duvar itmesi lideri geri itip ~72px'de asili birakiyordu
                    # (18:29 testi: mesafe 72'de kilit + pivot loop). <95px'de itmeyi
                    # 0.35'e kis -> hedef cekisi baskin -> lider hedefe ulasir.
                    if dist_final < 95.0:
                        rep_x *= 0.35
                        rep_y *= 0.35
                    target_angle = math.degrees(
                        math.atan2(wdy + rep_y, wdx + rep_x)) % 360
                    a_diff = angle_diff_deg(target_angle, langle)

                    # PIVOT ESIGI 60 -> 80 (16:46, kullanici: "engelden kacma cok
                    # keskin, sim gibi yumusak donsun"): 60'ta aci hatasi (kamera
                    # gecikmesiyle siser) 60'i asinca fwd=0 -> YERINDE PIVOT (keskin).
                    # 80'de lider 80 dereceye kadar ILERI bilesenle KAVISLE doner
                    # (sim gibi yumusak); sadece cok sert (>80) donuste pivot atar.
                    # SON YAKLASMA: hedefe <95px yakinken 130 -> hic pivot etmez,
                    # suzulup ARRIVAL_RADIUS'a girer (vardim-variyorum loop fix).
                    # ESIK 70->95: lider 72px'de (44 varis ile 70 arasi BOSLUKTA)
                    # takilip pivot loop'a giriyordu (18:29 testi); 95 bu banti kapsar.
                    # MID-ROUTE 80->90 (2026-06-14): dar koridorda (120px gate) APF
                    # itmesi 80'i asinca lider YERINDE PIVOT atip zigzag yapiyordu
                    # (135514 logu: jit 167, pivot reversal). 90'da daha genis aci
                    # KAVISLE alinir -> koridorda pivot-zigzag azalir (sim degil, gercek-test).
                    pdeg = 130.0 if dist_final < 95.0 else 90.0
                    left_cmd, right_cmd = compute_motor_commands(
                        a_diff, dist_final, is_catchup=False,
                        speed_scale=speed_scale * LEADER_SPEED_SCALE,  # SADECE lider %50 yavas
                        pd_id="leader", kd=LEADER_TURN_KD, kp=LEADER_TURN_KP,
                        pivot_deg=pdeg,
                    )

                    # SERT GUVENLIK: lider heading yonunde duvara girecekse ileriyi blokla.
                    # KILITLENME FIX: hedef tam onde (deadzone -> turn=0) + duvar blokluyorsa
                    # eski kod (0,0) basip SONSUZA DEK donuyordu. Artik turn~0 ise duvardan
                    # UZAGA dogru kararli bir pivot zorlanir -> lider kendini kurtarir.
                    if heading_blocked(lx, ly, langle, mission.walls):
                        turn_part = (left_cmd - right_cmd) / 2.0
                        if abs(turn_part) < 0.3:
                            if rep_x or rep_y:
                                away = math.degrees(math.atan2(rep_y, rep_x)) % 360
                                esc = angle_diff_deg(away, langle)
                            else:
                                esc = 90.0   # itme yoksa (kose tam ustunde) sola don
                            turn_part = 0.6 if esc > 0 else -0.6
                        left_cmd, right_cmd = turn_part, -turn_part

                    wp_info = f"WP {mission.current_wp + 1}/{len(wps)}" if wps else ""
                    status_text = (f"NAV {wp_info} | Mesafe:{int(dist_final)}px | "
                                   f"{formation_mode.upper()}")
                    status_color = (0, 255, 100)
                    if dist_final < SLOW_ZONE:
                        status_color = (0, 200, 255)
                else:
                    # Lider finise vardi -> DURUR, path donar. Takipciler kendi
                    # path'lerini bitirince TEK TEK parka gecer (asagida tetiklenir).
                    if not mission.arrived["flag"]:
                        mission.on_leader_arrived(lx, ly, langle, now, robot_states)
                    else:
                        mission.target_x = None
                        mission.target_y = None
                    status_text = "FINISTE | Takipciler path'i tamamliyor..."
                    status_color = (0, 255, 0)

            if not paused:
                robot_pwms[0] = (apply_deadband(left_cmd), apply_deadband(right_cmd))

            # Serit-tabanli ACC referans yonu: lider acisi (yoksa son bilinen)
            if robot_states[0]["angle"] is not None:
                leader_heading_deg = robot_states[0]["angle"]
            else:
                leader_heading_deg = last_known[0]["angle"]

            # Yumusak formasyon gecisi: slot degerlerini hedefe kademeli yaklastir
            morph_step = FORMATION_MORPH_RATE * dt
            for rid in FOLLOWER_IDS:
                tgt = FORMATIONS[formation_mode][rid]
                cur = current_slots[rid]
                for k in ("side_offset", "fb_offset", "target_dist"):
                    cur[k] = step_value(cur[k], tgt[k], morph_step)

            # ====== 8) TAKIPCI kontrol (islem sirasi: Lider->R1->R3->R2) ======
            trace = list(mission.follow_trace)   # tek paylasimli iz (bkz. GOREV DURUMU bolumu)

            # Kalkis tetigi icin: AKTIF takipcilerin kat ettigi yolu biriktir
            # (tracking sicramalari sayilmaz; predicted kareler atlanir)
            if mission.target_active:
                for rid in FOLLOWER_IDS:
                    if (mission.follower_activated[rid] and robot_states[rid]["found"]
                            and not robot_states[rid]["predicted"]):
                        cx_, cy_ = robot_states[rid]["x"], robot_states[rid]["y"]
                        lp_ = mission.last_xy[rid]
                        if lp_ is not None:
                            step_ = math.hypot(cx_ - lp_[0], cy_ - lp_[1])
                            if step_ < 60:
                                mission.traveled[rid] += step_
                        mission.last_xy[rid] = (cx_, cy_)

            # ===== UCGEN COHESION: 'dikey referans cizgi' = formasyon AGIRLIK
            # MERKEZI (COM). Her FOUND takipcinin gidis-yonu (trace bas->son)
            # boyunca ilerlemesini cikar; COM'a gore offset'i band'a normalize
            # et. one cikan apex asagida fuzzy ile frenlenir. SADECE split aktif.
            tri_lead = {}
            if mission.split and len(trace) >= 2:
                _ox, _oy = trace[0][0], trace[0][1]
                _ddx, _ddy = trace[-1][0] - _ox, trace[-1][1] - _oy
                _dmag = math.hypot(_ddx, _ddy) or 1.0
                _ux, _uy = _ddx / _dmag, _ddy / _dmag
                _progs = {r: (robot_states[r]["x"] - _ox) * _ux
                             + (robot_states[r]["y"] - _oy) * _uy
                          for r in FOLLOWER_IDS if robot_states[r]["found"]}
                if len(_progs) >= 2:
                    _com = sum(_progs.values()) / len(_progs)
                    _band = max(1.0, FORMATION_SPACING * TRIANGLE_LEAD_BAND_RATIO)
                    tri_lead = {r: (p - _com) / _band for r, p in _progs.items()}

            for rid in [1, 2, 3]:
                f_left_cmd, f_right_cmd = 0.0, 0.0
                acc_scale = 1.0
                tri_coh_stop = False   # UCGEN cohesion: band kenarinda ileri-kes (asagi)
                # TANI: durum cikarimi icin per-rid baslangic (sim portu)
                _dbg = DBG[rid]
                _dbg.clear()
                csc = 1.0          # collision olcegi (inner blok yazar; sabit kalir)
                is_catchup = False
                if not mission.follower_activated[rid]:
                    _dbg["state"] = "BEKLE"
                elif mission.parked[rid]:
                    _dbg["state"] = "PARK"

                if not paused and robot_states[rid]["found"] and mission.target_active:
                    fx = robot_states[rid]["x"]
                    fy = robot_states[rid]["y"]
                    fangle = robot_states[rid]["angle"]
                    target_dist = current_slots[rid]["target_dist"]
                    # Robot her zaman KENDI LINE slotunu (target_dist) korur — park'ta da reshuffle YOK,
                    # oldugu line dizilisinde kilitlenir. (Eski ucgen-yaklasim override'i kaldirildi.)

                    # --- SIRALI KALKIS: onceki robot KENDI YOLUNDA ~1 spacing KAT EDINCE sonraki kalkar ---
                    # (R1 -> R2 -> R3). Eski tetik "aradaki mesafe > 0.9*spacing" idi; baslangic
                    # diziliminde robotlar zaten uzaksa R1+R2 AYNI ANDA kalkiyordu (log: t=46.6s ikisi
                    # birden). traveled (kat edilen yol) 0'dan basladigi icin yaniltmaz.
                    if not mission.follower_activated[rid]:
                        prev = mission.prev_follower.get(rid)
                        if (prev and mission.follower_activated[prev]
                                and mission.activation_times[prev] is not None):
                            # KALKIS ARALIGI: LINE'da siki (~0.35); BOLUNMELI ucgende GENIS
                            # (~0.5=110px > temas 74px) -> onceki yan acilana kadar sonraki
                            # bekler, baslangic/merge'de karsilikli kilit olmaz (gercek log).
                            _gap = 0.50 if mission.split else 0.35
                            joined = mission.traveled[prev] > FORMATION_SPACING * _gap
                            timeout = (now - mission.activation_times[prev]) >= ACTIVATION_MAX_WAIT
                            if joined or timeout:
                                mission.follower_activated[rid] = True
                                mission.activation_times[rid] = now
                                print(f"[OK] Robot {rid} kalkti "
                                      f"({'onceki yol aldi' if joined else 'timeout'}).")

                    if mission.follower_activated[rid]:
                        # --- Hedef: lider varmissa sabit endpoint (PARK), hala yoldaysa path-based ---
                        use_endpoint = (mission.follower_final_targets[rid][0] is not None)
                        # BOLUNMELI ucgende yan robotlarin OZ rotasi (R3/R2); R1'de yok
                        own_rt = (mission.own_route.get(rid)
                                  if mission.split else None)

                        # --- UCGEN COHESION (centroid-offset BULANIK): bolunmeli
                        #     ucgende her robotun onunde takip edecegi robot YOK ->
                        #     line fuzzy'si calismiyordu, apex (R1) 78px/s firladi
                        #     (kullanici: 'rotaya en yakin robot cok erken hizlica
                        #     gitti'). COM'a gore +offset'e dayanan robot kademeli
                        #     frenlenir, geride kalan tam hizla yetisir -> ucgen
                        #     'dikey referans cizgi'nin yatay band'inda kalir.
                        #     Parkta (use_endpoint) KAPALI.
                        if mission.split and not use_endpoint and rid in tri_lead:
                            acc_scale = formation_offset_factor(tri_lead[rid])
                            _dbg["acc"] = acc_scale
                            _dbg["coh_lead"] = tri_lead[rid]   # SADECE cohesion uygulandiginda logla
                            # PWM TABANI fuzzy freni ETKISIZ birakiyor (acc 0.14'te bile
                            # ~taban hiz -> R1 one firliyordu). COZUM: ileri-bileseni DUTY
                            # ile PULSE et -> efektif hiz taban-ALTI -> R1 mumkun oldugunca
                            # YAVASLAR ama DURMAZ (kullanici istegi). duty=acc, taban 0.20.
                            if acc_scale < 0.85:
                                _duty = max(0.20, acc_scale)
                                _ph = _TRI_COH_PHASE.get(rid, 0.0) + _duty
                                if _ph >= 1.0:
                                    _ph -= 1.0          # bu kare ILERI (forward gecer)
                                else:
                                    tri_coh_stop = True  # bu kare pulse-off (ileri kes)
                                _TRI_COH_PHASE[rid] = _ph

                        # --- ACC (fuzzy fren): SADECE TAKIPTE ve SADECE LINE'da.
                        #     LINE'da 'onundeki bosluk' fuzzy'si; ucgen yukarida
                        #     centroid-cohesion ile ele alindi. Parkta KAPALI.
                        if not use_endpoint and not mission.split:
                            heading_ref = (leader_heading_deg
                                           if leader_heading_deg is not None else fangle)
                            fmode = formation_mode
                            if mission.fplan is not None:
                                fmode, _, _ = mission.fplan.query(rid, fx, fy)
                            if fmode != "triangle":
                                lane_w = FORMATION_SPACING * LANE_WIDTH_RATIO.get(fmode, 0.5)
                                d_ahead = nearest_ahead_dist(
                                    fx, fy, rid, last_known, now, heading_ref, lane_w)
                                if d_ahead != float("inf"):
                                    # FUZZY<->SLOT HIZALAMASI (16:56, R2 siki line'da
                                    # %47 frene giriyordu): fuzzy "guvenli" mesafesi
                                    # 165px ama slot hedefi SLOT_GAP_RATIO*S=121px ->
                                    # fuzzy slotun hedefini "cok yakin" sanip surekli
                                    # freniyordu. Girdiyi slot mesafesine normalize et:
                                    # slot mesafesi -> ratio~1.0 (guvenli, fren yok),
                                    # SADECE slottan yakinsa frenle. Temas guvenligi
                                    # binary collision_scale'de (90px) zaten var.
                                    d_norm = (d_ahead / SLOT_GAP_RATIO
                                              if fmode == "line" else d_ahead)
                                    acc_scale = fuzzy_speed_factor(d_norm, fmode)
                                    # TANI: 'onumdeki' robotun kimligi (fuzzy gorus)
                                    hr = math.radians(heading_ref)
                                    for o_ in last_known:
                                        if o_ == rid or last_known[o_]["x"] is None:
                                            continue
                                        dxo = last_known[o_]["x"] - fx
                                        dyo = last_known[o_]["y"] - fy
                                        lon = dxo * math.cos(hr) + dyo * math.sin(hr)
                                        lat = -dxo * math.sin(hr) + dyo * math.cos(hr)
                                        if (lon > 0 and abs(lat) < lane_w
                                                and abs(math.hypot(dxo, dyo) - d_ahead) < 1.0):
                                            _dbg["ahead"] = (o_, d_ahead)
                                            break

                        tx, ty = None, None
                        slot_i = None   # slot'un trace icindeki indeksi (carrot'u burada kes -> overrun yok)
                        slot_track = False   # s-bazli kayan slot aktif mi (sim portu)

                        if use_endpoint:
                            tx, ty = mission.follower_final_targets[rid]
                        elif own_rt:
                            # BOLUNMELI ucgen: yan robot OZ rotasinin sonuna gider
                            # (slot/yan-ofset YOK — rota zaten kendi seridi)
                            tx, ty = own_rt[-1][0], own_rt[-1][1]
                        elif len(trace) > 0:
                            # YENI AKIS: lider vardiktan sonra takipci izi SONUNA KADAR
                            # izler (slot tutmaz); park'a gecis asagida mesafeyle tetiklenir.
                            if mission.arrived["flag"]:
                                target_dist = 0.0
                            if target_dist <= 0:
                                rec_x, rec_y, rec_h = trace[-1]
                                slot_i = len(trace) - 1
                            else:
                                # FALLBACK: path target_dist'e ulasamiyorsa en eski noktaya
                                # clamp (None birakma! yoksa R2 hedefsiz kalir -> park tetiklenmez)
                                rec_x, rec_y, rec_h = trace[0]
                                slot_i = 0
                                cum_len = 0.0
                                for i in range(len(trace) - 1, 0, -1):
                                    dx_seg = trace[i][0] - trace[i - 1][0]
                                    dy_seg = trace[i][1] - trace[i - 1][1]
                                    cum_len += math.sqrt(dx_seg * dx_seg + dy_seg * dy_seg)
                                    if cum_len >= target_dist:
                                        rec_x, rec_y, rec_h = trace[i - 1]
                                        slot_i = i - 1
                                        break
                            # === S-BAZLI KAYAN SLOT (sim portu — "daha duzgun line"):
                            # izin SONU yerine ONCULUN iz-konumundan SLOT_GAP_RATIO*S
                            # geride KAYAN slot hedeflenir — slot onculle ilerler,
                            # hiz dogal esitlenir, akordeon yapisal olarak kalkar.
                            # Oncul parka ayrildiysa/parktaysa/tag kayipsa referans iz
                            # sonu. GERI YURUME YOK: slot kendi iz-konumumun gerisine
                            # dusemez (sim bulgusu: R2 rotanin basina geri donuyordu).
                            if (mission.arrived["flag"] and not mission.split
                                    and len(trace) >= 2):
                                cum_s = [0.0]
                                for qi in range(1, len(trace)):
                                    cum_s.append(cum_s[-1] + math.hypot(
                                        trace[qi][0] - trace[qi - 1][0],
                                        trace[qi][1] - trace[qi - 1][1]))
                                total_s = cum_s[-1]
                                pred = {3: 0, 1: 3, 2: 1}[rid]
                                pl_ = last_known.get(pred)
                                if (pred == 0 or mission.parked.get(pred)
                                        or mission.follower_final_targets.get(
                                            pred, (None, None))[0] is not None
                                        or pl_ is None or pl_["x"] is None):
                                    s_pred = total_s
                                else:
                                    s_pred = route_arc_s(trace, pl_["x"], pl_["y"])
                                s_t = max(0.0, s_pred -
                                          SLOT_GAP_RATIO * FORMATION_SPACING)
                                s_self = route_arc_s(trace, fx, fy)
                                s_t = min(total_s, max(s_t, s_self))
                                j = 0
                                while j < len(cum_s) - 1 and cum_s[j + 1] < s_t:
                                    j += 1
                                rec_x, rec_y, rec_h = trace[j]
                                slot_i = j
                                slot_track = True
                                _dbg["slot"] = (rec_x, rec_y)   # TANI: kayan slot
                            side_offset = current_slots[rid]["side_offset"]
                            fb_offset = current_slots[rid]["fb_offset"]
                            # Yan-ofset yonu: segment acisi (rec_h) KOSEDE ziplar -> hedef
                            # isinlanir/wobble. Liderin YUMUSAK acisini kullan (sim'deki gibi).
                            off_h = (leader_heading_deg
                                     if leader_heading_deg is not None else rec_h)
                            rad_h = math.radians(rec_h)   # fore-aft icin yerel yon
                            offset_rad = math.radians(off_h) + (math.pi / 2.0)
                            tx = rec_x + side_offset * math.cos(offset_rad) + fb_offset * math.cos(rad_h)
                            ty = rec_y + side_offset * math.sin(offset_rad) + fb_offset * math.sin(rad_h)

                        if tx is not None:
                            draw_follower_target(frame, rid, fx, fy, tx, ty)
                            _dbg["tgt"] = (tx, ty)   # TANI/LOG: hedef nokta
                            fdx, fdy = tx - fx, ty - fy
                            fdist = math.hypot(fdx, fdy)

                            # === KONVOY UYUMU (sim portu — WEBOTS USULU ORANTILI
                            # YAVASLAMA): fuzzy sadece ARKADAKINI frenler, onden kacani
                            # kimse tutmaz -> akordeon (15:46: R3-R1 483px). Kademeli
                            # PWM dusurme ISE YARAMAZ (min-PWM tabani kademeyi ezer) ->
                            # yavaslama ZAMAN-BOLMELI: 1.2 sn'lik periyodun duty
                            # kadarinda yuru, kalaninda mini-duraklama. Aralik acildikca
                            # duty duser (taban 0.35: asla heykel gibi durmaz; tam durus
                            # SADECE acil fren/carpisma katmaninda). UCGENDE KAPALI.
                            coh_stop = False
                            if (not use_endpoint and own_rt is None
                                    and not mission.split):
                                succ = {3: 1, 1: 2}.get(rid)
                                if (succ and mission.follower_activated.get(succ)
                                        and not mission.parked[succ]
                                        and mission.follower_final_targets[succ][0] is None):
                                    sp_ = last_known[succ]
                                    if sp_["x"] is not None:
                                        gap_ = math.hypot(sp_["x"] - fx, sp_["y"] - fy)
                                        rel_ = (SLOT_GAP_RATIO + 0.15) * FORMATION_SPACING
                                        stp_ = (SLOT_GAP_RATIO + 0.55) * FORMATION_SPACING
                                        if gap_ > rel_:
                                            duty_ = max(0.35, 1.0 - 0.65 * (gap_ - rel_)
                                                        / max(1.0, stp_ - rel_))
                                            _dbg["coh_gap"] = gap_
                                            if (now % 1.2) > 1.2 * duty_:
                                                coh_stop = True
                                                _dbg["state"] = "COHESION %%%d" % int(100 * duty_)
                                                # beklerken takilma sigortasi islemesin
                                                mission.park_progress.pop(rid, None)

                            # FINISE YAKLASAN TAKIPCI PARK POZISYONUNA GECER (kullanici akisi):
                            # iz takibinden cik, kendi slotuna (R3 sag, R2 sol, R1 arka) don.
                            if (mission.arrived["flag"] and not use_endpoint and not coh_stop
                                    and mission.follower_final_targets[rid][0] is None):
                                d_fin = math.hypot(mission.arrived["x"] - fx,
                                                   mission.arrived["y"] - fy)
                                if d_fin < PARK_ENTER_DIST and mission.park_slots:
                                    best_i, best_d = -1, 1e18
                                    for si, sp in enumerate(mission.park_slots):
                                        if mission.slot_taken[si]:
                                            continue
                                        dd = (sp[0] - fx) ** 2 + (sp[1] - fy) ** 2
                                        if dd < best_d:
                                            best_d, best_i = dd, si
                                    if best_i >= 0:
                                        mission.slot_taken[best_i] = True
                                        mission.follower_final_targets[rid] = mission.park_slots[best_i]
                                        print(f"[OK] R{rid} finise yaklasti -> EN YAKIN bos "
                                              f"slota ({best_i + 1}) gidiyor.")
                                        use_endpoint = True
                                        tx, ty = mission.follower_final_targets[rid]
                                        fdx, fdy = tx - fx, ty - fy
                                        fdist = math.hypot(fdx, fdy)

                            # === WEBOTS USULU KAVSAK PROTOKOLU (sim portu;
                            # transition_wait_mode'un karsiligi) ===
                            # Iki yan robot ayni birlesme kavsagina yaklasirken:
                            # KAVSAGA YAKIN OLAN GECER, digeri TAM DURUR (sifir komut —
                            # surunme/pivot yok). Rakip kavsagi gecince (veya parka
                            # ayrilinca) bekleyen kaldigi yerden devam eder.
                            merge_stop = False
                            if (mission.split and not use_endpoint and own_rt
                                    and mission.merge_pts):
                                rival = 2 if rid == 3 else 3
                                rb = last_known[rival]
                                if rb["x"] is not None:
                                    for (mqx, mqy, mqs) in mission.merge_pts:
                                        dme = math.hypot(mqx - fx, mqy - fy)
                                        if dme > 220.0:
                                            continue
                                        if (mission.parked[rival] or
                                                mission.follower_final_targets[rival][0]
                                                is not None):
                                            break   # rakip parka ayrildi -> kavsak bos
                                        drv = math.hypot(mqx - rb["x"], mqy - rb["y"])
                                        rival_past = (drv > 120.0 and route_arc_s(
                                            mission.planned_route, rb["x"], rb["y"])
                                            > mqs + 40.0)
                                        contesting = (drv < 220.0) and not rival_past
                                        # Yakin olan gecer; esitlikte R2 (yuksek oncelik)
                                        i_lose = ((dme > drv - 15.0) if rid == 3
                                                  else (dme > drv + 15.0))
                                        if contesting and i_lose:
                                            # FUZZY-VARI ORANTILI KAVSAK HIZI (sim
                                            # portu — kullanici: "durdurma, yavaslat"):
                                            # kavsaga uzakken %80 duty ile hizli
                                            # yaklas, yaklastikca %35'e in. Sim A/B
                                            # (3 tohum): kavsak bekleme payi ~%0,
                                            # park 3/3. Temas guvenligi binary frende.
                                            mission.park_progress.pop(rid, None)
                                            mission.stuck.pop(rid, None)
                                            duty_m = 0.35 + 0.45 * min(1.0, dme / 220.0)
                                            _dbg["merge"] = (mqx, mqy)
                                            _dbg["mtxt"] = "R%d bekler (%d>rakip %d)" % (
                                                rid, dme, drv)
                                            if (now % 1.2) > 1.2 * duty_m:
                                                merge_stop = True
                                                _dbg["state"] = "KAVSAK BEKLE"
                                        break

                            # SLOT TUTMA BANDI (sim portu): slotuna oturan bekler
                            # (min-PWM titremesi yapmasin); beklerken takilma
                            # sigortasi sayaci sifirlanir (oncul yavassa kilit yok).
                            slot_hold = False
                            if slot_track and not use_endpoint and fdist < 14.0:
                                mission.park_progress.pop(rid, None)
                                mission.stuck.pop(rid, None)
                                slot_hold = True
                                _dbg["state"] = "SLOT TUT"

                            # === TAKILMA KURTARMA MERDIVENI (sim portu — kullanici
                            # tasarimi: "kilitleme, once baska noktaya gucu artirarak
                            # yonelsin"): 5 sn'dir NET yer degistirmeyen robot
                            # (kipirdanma dahil — sallanma sigortayi kandiramaz):
                            # 1) 0.7 sn GERI cekil, 2) hedefin 90 derece yanindaki
                            # kacis noktasina guc artisiyla git (3.5 sn, yon secimi
                            # duvara uzak yan — acil SISTEM manevrasi), 3) olmadi obur
                            # yandan, 4) 3. basarisizlikta SON CARE kilidi.
                            # PARK TRAFIGI ISTISNASI: parkli/parka-ayrilmis robotun
                            # veya liderin dibinde durmak mesru yol vermedir.
                            rec_cmd = None
                            if (STUCK_RECOVERY
                                    and mission.arrived["flag"] and not mission.parked[rid]
                                    and not (slot_hold or merge_stop or coh_stop)):
                                st_ = mission.stuck.get(rid)
                                if st_ is None:
                                    st_ = mission.stuck[rid] = {
                                        "ax": fx, "ay": fy, "t": now, "fails": 0,
                                        "mode": None, "until": 0.0, "esc": None}
                                if st_["mode"] == "rev":
                                    if now < st_["until"]:
                                        rec_cmd = (-26.0, -26.0)
                                    else:
                                        ux_, uy_ = tx - fx, ty - fy
                                        mm_ = math.hypot(ux_, uy_) or 1.0
                                        first = -1.0 if st_["fails"] % 2 == 0 else +1.0
                                        cands = []
                                        for side_ in (first, -first):
                                            exy = (fx + (-uy_ / mm_) * side_ * 110.0
                                                   + (ux_ / mm_) * 20.0,
                                                   fy + (ux_ / mm_) * side_ * 110.0
                                                   + (uy_ / mm_) * 20.0)
                                            clr = (wall_clear_dist(exy[0], exy[1],
                                                                   mission.walls)
                                                   if mission.walls else 999.0)
                                            cands.append(
                                                (clr + (5.0 if side_ == first else 0.0),
                                                 exy))
                                        cands.sort(reverse=True)
                                        st_["esc"] = cands[0][1]
                                        st_["mode"] = "esc"
                                        st_["until"] = now + 3.5
                                if st_["mode"] == "esc" and rec_cmd is None:
                                    ex_, ey_ = st_["esc"]
                                    ed_ = math.hypot(ex_ - fx, ey_ - fy)
                                    if now >= st_["until"] or ed_ < 30.0:
                                        st_["mode"] = None
                                        st_["ax"], st_["ay"], st_["t"] = fx, fy, now
                                    else:
                                        ad_ = angle_diff_deg(math.degrees(
                                            math.atan2(ey_ - fy, ex_ - fx)) % 360,
                                            fangle)
                                        rec_cmd = compute_motor_commands(
                                            ad_, ed_, is_catchup=True,
                                            speed_scale=speed_scale * 1.25
                                            * FOLLOWER_SPEED_SCALE,
                                            pd_id=f"esc{rid}", kd=FOLLOWER_TURN_KD,
                                            kp=FOLLOWER_TURN_KP)
                                if st_["mode"] is None and rec_cmd is None:
                                    near_traffic = False
                                    for o_ in (0, 1, 2, 3):
                                        if o_ == rid:
                                            continue
                                        if (o_ == 0 or mission.parked.get(o_)
                                                or mission.follower_final_targets.get(
                                                    o_, (None, None))[0] is not None):
                                            ol_ = last_known[o_]
                                            if (ol_["x"] is not None and math.hypot(
                                                    ol_["x"] - fx, ol_["y"] - fy) < 130.0):
                                                near_traffic = True
                                                break
                                    if near_traffic:
                                        st_["ax"], st_["ay"], st_["t"] = fx, fy, now
                                    elif math.hypot(fx - st_["ax"],
                                                    fy - st_["ay"]) > 45.0:
                                        st_["ax"], st_["ay"], st_["t"] = fx, fy, now
                                        st_["fails"] = 0
                                    elif now - st_["t"] > 5.0:
                                        # SON-CARE KILIDI KALDIRILDI (kullanici
                                        # karari): robot pes etmez, yanlari
                                        # donusumlu deneyerek ugrasir. (Hic
                                        # kipirdayamayani eski sigorta yakalar.)
                                        st_["fails"] += 1
                                        print(f"[KURTARMA] R{rid} takildi "
                                              f"(deneme {st_['fails']}) -> "
                                              f"geri cekil + yan kacis")
                                        st_["mode"] = "rev"
                                        st_["until"] = now + 0.7
                                        rec_cmd = (-26.0, -26.0)

                            # --- PARK (SIM-TARZI, IKI ASAMALI): slot hedefi atanmis robot
                            #     (use_endpoint) slotuna varinca kilitlenir. SIGORTA artik
                            #     ILERLEME bazli: yol aldigi surece ASLA kesilmez (uzaktan
                            #     gelen robot yolda kilitlenmez — 18:27 testindeki hata);
                            #     sadece PARK_STUCK_SEC boyunca ilerleyemezse (engel/tag
                            #     kaybi) oldugu yerde kilitlenir. R1 slotunu yanlar bittikten
                            #     SONRA alir; o ana kadar line slotunda bekler (kilitlenmez).
                            if mission.arrived["flag"] and not mission.parked[rid]:
                                settled = use_endpoint and fdist < PARK_ARRIVAL_RADIUS
                                stuck = False
                                # Sigorta artik SLOT-ONCESINDE de gecerli: dar koridorda
                                # onceki robot tikadiysa PARK_ENTER esigine hic ulasamayabilir
                                # (sim bulgusu) -> daha uzun pencereyle yerinde kilitlenir.
                                trv = mission.traveled[rid]
                                pp = mission.park_progress.get(rid)
                                if pp is None or trv - pp[0] > PARK_STUCK_MIN_PX:
                                    mission.park_progress[rid] = [trv, now]
                                elif now - pp[1] > (PARK_STUCK_SEC if use_endpoint
                                                    else PARK_STUCK_SEC * 2):
                                    stuck = True
                                if settled or stuck:
                                    mission.parked[rid] = True
                                    print(f"[OK] Robot {rid} -> park"
                                          f"{' (takildi, oldugu yerde)' if stuck and not settled else ''}.")

                            if mission.parked[rid] or slot_hold:
                                if mission.parked[rid]:
                                    _dbg["state"] = "PARK"
                                # KILITLI (park) veya SLOTUNDA — TAM DUR.
                                # merge/cohesion duraklamalari artik TAM DUR DEGIL:
                                # asagida sadece ILERI bileseni kesilir, DONUS
                                # SERBEST kalir (19:44 logu: tam-dur duraklar her
                                # kalkista 0.2sn gecikmeli kafa vurusu -> zikzak;
                                # R3 24sn'de 31 dur-kalk yapmisti).
                                f_left_cmd, f_right_cmd = 0.0, 0.0
                            elif rec_cmd is not None:
                                # KURTARMA MANEVRASI: geri cekilme / yan kacis
                                # komutlari normal takibi gecersiz kilar
                                _dbg["state"] = "KURTARMA"
                                f_left_cmd, f_right_cmd = rec_cmd
                            else:
                                # YONLENDIRME (pure pursuit): ILERIDEKI havuc noktasina nisan
                                # al -> cizgiyi siki takip eder. HIZ slot mesafesi (fdist) ile
                                # ayarlanir (formasyon araligini korur).
                                aim_pt = None
                                if not use_endpoint:
                                    # Havuc tavani 48px (sim portu): duvar BILGISI degil,
                                    # takip sikiligi — uzun havuc koseyi kirpiyordu (kor
                                    # takipci testinde -14px gomulme). Duvar-bazli
                                    # kisaltma KALDIRILDI (takipciler engel gormez).
                                    la = min(48.0, adaptive_lookahead(trackers[rid].speed()))
                                    # CROSS-TRACK: rotaya yanal hata buyukse lookahead'i KISALT
                                    # -> dagiNik baslangictan/sapinca cizgiye HIZLI (dik) oturur.
                                    # own_rt varsa ona, yoksa trace'e gore. (T-kapida LINE icin sart:
                                    # eskiden R2 merkez own_route'una yakinsayamiyordu, 75px sapiyordu.)
                                    _xref = own_rt if own_rt else (trace if len(trace) >= 2 else None)
                                    if _xref and len(_xref) >= 2:
                                        _xte = min(pt_seg_dist((fx, fy),
                                                   (_xref[k][0], _xref[k][1]),
                                                   (_xref[k + 1][0], _xref[k + 1][1]))
                                                   for k in range(len(_xref) - 1))
                                        _xte = 0.35 * _xte + 0.65 * _XTE_EMA.get(rid, _xte)
                                        _XTE_EMA[rid] = _xte
                                        la = max(XTRACK_LA_MIN,
                                                 la * crosstrack_lookahead_factor(_xte / XTRACK_REF))
                                    if own_rt:
                                        # BOLUNMELI ucgen: havuc OZ rotada (ofset yok)
                                        cr = carrot_on_path(fx, fy, own_rt, la)
                                        if cr is not None:
                                            aim_pt = (cr[0], cr[1])
                                    else:
                                        # OVERRUN ENGELI: carrot'u kendi SLOT'unda kes -> follower
                                        # slotunun OTESINE (lidere dogru) nisan ALMAZ. Boylece slotta
                                        # durur (fdist->0), overrun olmaz, park tetiklenir. Uzaktayken
                                        # sub uzun -> normal yumusak takip (degisiklik yok).
                                        sub = (trace[:slot_i + 1]
                                               if (slot_i is not None and slot_i + 1 >= 2) else trace)
                                        cr = carrot_on_path(fx, fy, sub, la)
                                        if cr is not None:
                                            ccx, ccy, cch = cr
                                            if mission.fplan is not None and not mission.split:
                                                # ROTA-TABANLI PLAN (sim portu): serit ofseti
                                                # o noktadaki BOSLUGA gore (R1 sol, R2 sag,
                                                # R3 merkez; dar yerde otomatik LINE=0)
                                                _, foff, fnv = mission.fplan.query(rid, fx, fy)
                                                aim_pt = (ccx + fnv[0] * foff,
                                                          ccy + fnv[1] * foff)
                                            else:
                                                so = current_slots[rid]["side_offset"]
                                                # Havuc segment acisi kosede ziplar -> liderin
                                                # yumusak acisini kullan (wobble'i keser).
                                                oh = (leader_heading_deg
                                                      if leader_heading_deg is not None else cch)
                                                orad = math.radians(oh) + (math.pi / 2.0)
                                                aim_pt = (ccx + so * math.cos(orad),
                                                          ccy + so * math.sin(orad))
                                if aim_pt is not None:
                                    _dbg["carrot"] = aim_pt   # TANI: pure-pursuit havuc
                                    adx, ady = aim_pt[0] - fx, aim_pt[1] - fy
                                else:
                                    adx, ady = fdx, fdy
                                amag = math.hypot(adx, ady) or 1.0
                                adx, ady = adx / amag, ady / amag   # birim hedef yonu
                                # === TAKIPCILER ENGELLERI GORMEZ (kullanici karari —
                                # Webots saflagi, sim portu): duvar itmesi YOK, duvar-onu
                                # kacisi YOK. Takipci SADECE verilen rotayi izler (rota
                                # zaten 46px duvar marjiyla planli). Robot-robot bilgisi
                                # AYNEN devam: vorteks + ACC + binary fren + kavsak.
                                rr_x, rr_y = robot_repulsion(rid, fx, fy,
                                                             robot_states, last_known, now,
                                                             goal_dir=(adx, ady))
                                ftarget_angle = math.degrees(
                                    math.atan2(ady + rr_y, adx + rr_x)) % 360
                                fangle_diff = angle_diff_deg(ftarget_angle, fangle)
                                # CATCHUP: geride kalinca yetisir; ramming'i collision_scale onler
                                is_catchup = fdist > FORMATION_SPACING * 0.6
                                f_left_cmd, f_right_cmd = compute_motor_commands(
                                    fangle_diff, fdist,
                                    is_catchup=is_catchup,
                                    speed_scale=speed_scale * acc_scale * FOLLOWER_SPEED_SCALE,
                                    pd_id=f"follower{rid}", kd=FOLLOWER_TURN_KD,
                                    kp=FOLLOWER_TURN_KP, pivot_deg=FOLLOWER_PIVOT_DEG,
                                )
                                # Formasyon yerine cok yakinsa dur (creep/overshoot engeli)
                                if not use_endpoint and fdist < 10:
                                    f_left_cmd, f_right_cmd = 0.0, 0.0

                            # CARPISMA ONLEME (YUMUSAK): kademeli yavasla. PARK modunda da ACIK ama
                            # DAR bant (75px) -> lidere/parkmis robota carpmaz AMA 130px arali slota girebilir.
                            # Takipte genis bant (145px). float carpim (int kuantalama yumusakligi oldurur).
                            if not mission.parked[rid] and rec_cmd is None:
                                # parkta dar bant (105px > temas 78px, < slot araligi 130px) -> carpmaz ama slota girer
                                # (kurtarma manevrasi collision'dan muaf — sim ile ayni)
                                stop_dist = 105.0 if use_endpoint else COLLISION_STOP_DIST
                                mvx_h = math.cos(math.radians(fangle))
                                mvy_h = math.sin(math.radians(fangle))
                                # 'BIRI BEKLESIN, DIGERI DEVAM ETSIN' (sim portu —
                                # birlesme kilitlenmesine care): benden DUSUK oncelikli
                                # robotlar BANA yol verir; cok yakin (<90px) olmadikca
                                # benim fren hesabima GIRMEZLER. Yuksek oncelikli riske
                                # girip gecer, dusuk bekler — simetrik kitlenme yok.
                                # (90px: temas + suzulme payi; vorteks zaten kaydirir.)
                                my_pr = COLLISION_PRIORITY.index(rid)
                                cs_states, cs_lk = {}, {}
                                for o in last_known:
                                    if (o != rid and o != 0
                                            and COLLISION_PRIORITY.index(o) > my_pr
                                            and last_known[o]["x"] is not None):
                                        dxo = last_known[o]["x"] - fx
                                        dyo = last_known[o]["y"] - fy
                                        if math.hypot(dxo, dyo) > 90.0:
                                            continue   # dusuk oncelikli ve uzak: o bana yol verir
                                    cs_states[o] = robot_states[o]
                                    cs_lk[o] = last_known[o]
                                # SIRA-VERME (kullanici: ucgen biterken yanlardan biri
                                # beklesin): split'te BEN yuksek oncelikli yansam, benden
                                # DUSUK oncelikli yan robot icin graded anti-ram'i atla ->
                                # ben gecerim, o bekler (ikisi birden kilitlenmez). Temas
                                # (<74px) hard-stop korunur. LINE/park-disi/lider etkilenmez.
                                anti_ram_skip = None
                                if mission.split and rid in mission.side_ids:
                                    anti_ram_skip = {o for o in mission.side_ids
                                                     if o != rid and COLLISION_PRIORITY.index(o)
                                                     > COLLISION_PRIORITY.index(rid)}
                                csc = collision_scale(rid, fx, fy, mvx_h, mvy_h,
                                                      cs_states, cs_lk, now,
                                                      stop_dist, anti_ram_skip=anti_ram_skip)
                                # TANI: panel/rozet icin sinyaller + nihai durum
                                # cikarimi (merge/coh/slot/park zaten yazdiysa
                                # dokunma — onlar daha aciklayici).
                                _dbg["csc"] = csc
                                _dbg["acc"] = acc_scale
                                _dbg["stop_dist"] = stop_dist
                                _dbg["fdist"] = fdist
                                _dbg["adiff"] = fangle_diff
                                _dbg["vortex"] = (rr_x, rr_y)
                                if "state" not in _dbg:
                                    if use_endpoint:
                                        _dbg["state"] = "PARKA GIT"
                                    elif csc <= 0.01:
                                        _dbg["state"] = "CARPISMA FRENI"
                                    elif csc < 1.0:
                                        _dbg["state"] = "FREN %%%d" % int(100 * csc)
                                    elif acc_scale < 0.9:
                                        _dbg["state"] = "FUZZY %%%d" % int(100 * acc_scale)
                                    elif is_catchup:
                                        _dbg["state"] = "CATCHUP"
                                    else:
                                        _dbg["state"] = "TAKIP"
                                # csc SADECE ILERI bileseni keser; DONUS SERBEST kalir ->
                                # robot durdugunda bile kacinma yonune donebilir, donup
                                # kalmaz (eski kod iki tekeri birden sifirliyordu = donma).
                                fwd_p = (f_left_cmd + f_right_cmd) / 2.0
                                turn_p = (f_left_cmd - f_right_cmd) / 2.0
                                fwd_p *= csc
                                if merge_stop or coh_stop or tri_coh_stop:
                                    # duty duraklamasi: ileri yok. DONUS KAPISI
                                    # (sim A/B: serbest donus duraklamada %38-41
                                    # pivot segirmesi uretti): donus ancak
                                    # belirgin sapmada (>20 der) serbest —
                                    # hizalanma faydasi kalir, segirme kalkar.
                                    fwd_p = 0.0
                                    if abs(fangle_diff) < 20.0:
                                        turn_p = 0.0
                                f_left_cmd = fwd_p + turn_p
                                f_right_cmd = fwd_p - turn_p

                if paused:
                    robot_pwms[rid] = (0, 0)
                else:
                    mp = FOLLOWER_MIN_PWM + ROBOT_PWM_TRIM.get(rid, 0)
                    robot_pwms[rid] = (apply_deadband(f_left_cmd, mp),
                                       apply_deadband(f_right_cmd, mp))

            # ====== 9) Komutlari TX thread'ine ver (cadence/invert orada) ======
            writer.set_pwms(robot_pwms)

            # ====== 9b) TANI: kosu karnesi (RUN) + hiz gecmisi (salt gozlem) ======
            LAST_CMDS.update(robot_pwms)
            if (not SPDH or now - SPDH[-1][0] > 0.3):
                SPDH.append((now, trackers[1].speed(), trackers[2].speed(),
                             trackers[3].speed()))
            if mission.target_active and RUN["key"] != id(mission.planned_route):
                RUN.update(key=id(mission.planned_route), t0=now, t_arr=None,
                           done=False, piv={1: 0, 2: 0, 3: 0}, idle={1: 0, 2: 0, 3: 0},
                           tot={1: 0, 2: 0, 3: 0}, d31=[], d12=[], ldev=[])
            if not RUN["done"] and mission.target_active:
                if mission.arrived["flag"]:
                    if RUN["t_arr"] is None:
                        RUN["t_arr"] = now
                    for r in (1, 2, 3):
                        if mission.follower_activated[r] and not mission.parked[r]:
                            RUN["tot"][r] += 1
                            pl, pr = robot_pwms[r]
                            if pl * pr < 0:
                                RUN["piv"][r] += 1
                            elif pl == 0 and pr == 0:
                                RUN["idle"][r] += 1
                    rs = robot_states
                    if rs[3]["found"] and rs[1]["found"]:
                        RUN["d31"].append(math.hypot(rs[3]["x"] - rs[1]["x"],
                                                     rs[3]["y"] - rs[1]["y"]))
                    if rs[1]["found"] and rs[2]["found"]:
                        RUN["d12"].append(math.hypot(rs[1]["x"] - rs[2]["x"],
                                                     rs[1]["y"] - rs[2]["y"]))
                if mission.arrived["flag"] and all(mission.parked.values()):
                    g31 = sorted(RUN["d31"]) or [0]
                    g12 = sorted(RUN["d12"]) or [0]
                    REPORT["prev"] = REPORT["cur"]
                    REPORT["cur"] = {
                        "sure": now - RUN["t0"],
                        "lider": (RUN["t_arr"] or now) - RUN["t0"],
                        "d31": g31[len(g31) // 2], "d12": g12[len(g12) // 2],
                        "sap_o": 0.0, "sap_m": 0.0,
                        "piv": {r: 100 * RUN["piv"][r] // max(1, RUN["tot"][r])
                                for r in (1, 2, 3)},
                        "idle": {r: 100 * RUN["idle"][r] // max(1, RUN["tot"][r])
                                 for r in (1, 2, 3)}}
                    RUN["done"] = True
                    WINDOWS["RAPOR"]["open"] = True
                    print("[RAPOR] kosu karnesi hazir (%.0fs)" % REPORT["cur"]["sure"])
                    # OTOMATIK MATLAB EXPORT: .mat (veri) + _ciz.m (cizim scripti).
                    # Ayri modul + try/except -> hata olsa bile gorev/program cokmez.
                    try:
                        _log_file.flush()
                        import run_export
                        res = run_export.export(log_name, px_per_cm,
                                                FORMATION_SPACING)
                        if res:
                            print("[EXPORT] MATLAB dosyalari: %s | %s"
                                  % (os.path.basename(res[0]),
                                     os.path.basename(res[1])))
                    except Exception as _e:
                        print("[EXPORT] atlandi (%s)" % _e)
                    # 3/3 PARK -> OTO-CIKIS (kullanici istegi): son durumu/RAPOR'u
                    # gormen icin AUTO_EXIT_GRACE sn ekranda kalir, sonra kapanir.
                    if auto_exit_t is None:
                        auto_exit_t = now
                        print("[CIKIS] Gorev tamam — %d sn sonra otomatik kapaniyor "
                              "(beklemeden cikmak icin Q)." % int(AUTO_EXIT_GRACE))

            # ====== 10) HUD + log ======
            # TANI KATMANI ([G]) — guncel-kare DBG ile, HUD'dan ONCE (altta kalir)
            if analyze:
                draw_diagnostics(frame, mission, robot_states, last_known, px_per_cm)
            draw_ruler(frame, px_per_cm)
            cam_warn = "KAMERA GECIKMELI!" if (now - t_frame) > 0.25 else None
            draw_hud(frame, status_text, status_color, robot_pwms, speed_scale,
                        px_per_cm, fps_display, writer.status_text(), cam_warn,
                        tag_stats_text)
            draw_windows(frame, mission, robot_states, last_known)   # [T] paneller

            if now - last_log_time >= 0.3:
                last_log_time = now

                def _p(rid):
                    s = robot_states[rid]
                    if s["found"]:
                        return f"{int(s['x'])},{int(s['y'])},{int(s['angle'])}"
                    return ",,"

                def _d(a, b):
                    sa, sb = robot_states[a], robot_states[b]
                    if sa["found"] and sb["found"]:
                        return f"{math.hypot(sa['x'] - sb['x'], sa['y'] - sb['y']):.0f}"
                    return "-1"

                act = mission.follower_activated
                prk = mission.parked

                def _st(rid):   # TANI durum metni (virgul/yeni-satir guvenli)
                    return str(DBG[rid].get("state", "-")).replace(",", ";")

                # --- B-LISTE yardimcilari (DBG/tracker/olceklerden) ---
                def _v(rid):    # anlik hiz (px/s) — Kalman'dan
                    return "%.1f" % trackers[rid].speed()

                def _acc(rid):
                    return "%.3f" % DBG[rid].get("acc", 1.0)

                def _dah(rid):  # fuzzy girdi mesafesi (yoksa -1)
                    ah = DBG[rid].get("ahead")
                    return "%.0f" % ah[1] if ah else "-1"

                def _csc(rid):
                    return "%.2f" % DBG[rid].get("csc", 1.0)

                def _tgt(rid):  # hedef nokta (yoksa ,)
                    tg = DBG[rid].get("tgt")
                    return f"{tg[0]:.0f},{tg[1]:.0f}" if tg else ","

                def _lead(rid):  # UCGEN cohesion girdisi — SADECE cohesion uygulanan
                    # karede (park/line'da bos -> grafikte acc=1.0 kirliligi olmaz)
                    cl = DBG[rid].get("coh_lead")
                    return f"{cl:.3f}" if cl is not None else ""

                # lider rota sapmasi (px): son bilinen lider -> planli rota
                ldev = -1.0
                lk0 = last_known[0]
                if mission.planned_route and len(mission.planned_route) >= 2 \
                        and lk0["x"] is not None:
                    ldev = min(pt_seg_dist((lk0["x"], lk0["y"]),
                                           mission.planned_route[qi],
                                           mission.planned_route[qi + 1])
                               for qi in range(len(mission.planned_route) - 1))

                row = (f"{now:.2f},{_p(0)},{_p(1)},{_p(2)},{_p(3)},"
                       f"{_d(0,2)},{_d(0,1)},{_d(0,3)},{_d(1,2)},{_d(1,3)},{_d(2,3)},"
                       f"{int(act[2])},{int(act[1])},{int(act[3])},"
                       f"{mission.leader_dist_since_target:.0f},{formation_mode},"
                       f"{int(prk[1])},{int(prk[3])},{int(prk[2])},{status_text[:30]},"
                       f"{robot_pwms[0][0]},{robot_pwms[0][1]},"
                       f"{robot_pwms[1][0]},{robot_pwms[1][1]},"
                       f"{robot_pwms[2][0]},{robot_pwms[2][1]},"
                       f"{robot_pwms[3][0]},{robot_pwms[3][1]},"
                       f"{_st(1)},{_st(2)},{_st(3)},"
                       f"{_v(0)},{_v(1)},{_v(2)},{_v(3)},"
                       f"{_acc(1)},{_acc(2)},{_acc(3)},"
                       f"{_dah(1)},{_dah(2)},{_dah(3)},"
                       f"{_csc(1)},{_csc(2)},{_csc(3)},"
                       f"{_tgt(1)},{_tgt(2)},{_tgt(3)},"
                       f"{ldev:.1f},{int(mission.split)},"
                       f"{px_per_cm if px_per_cm else -1:.2f},{fps_display},"
                       f"{_lead(1)},{_lead(2)},{_lead(3)}\n")
                _log_file.write(row)
                _log_file.flush()

            cv2.imshow(WINDOW_NAME, frame)
            last_rendered = frame

            if handle_keys():
                break
            # 3/3 park sonrasi otomatik cikis (grace suresi dolunca)
            if auto_exit_t is not None and now - auto_exit_t > AUTO_EXIT_GRACE:
                print("[CIKIS] Otomatik kapaniyor.")
                break

    finally:
        print("\nKapatiliyor...")
        writer.stop()      # 3 tur 0 basar, portu kapatir
        camera.stop()      # reader thread join + release
        cv2.destroyAllWindows()
        _log_file.flush()
        _log_file.close()
        # KAPANISTA MATLAB EXPORT (Q veya cikis): tum oturum logundan .mat + .m
        # uretir (gorev yarida kalsa bile). Gorev tamamlandiysa zaten uretilmisti;
        # ayni dosya adina yazar -> tam-oturum surumuyle yeniler. Hata yutulur.
        try:
            import run_export
            res = run_export.export(log_name, px_per_cm, FORMATION_SPACING)
            if res:
                print("[EXPORT] MATLAB dosyalari (kapanis): %s | %s"
                      % (os.path.basename(res[0]), os.path.basename(res[1])))
        except Exception as _e:
            print("[EXPORT] kapanista atlandi (%s)" % _e)
        print("[OK] Sistem kapatildi.")


if __name__ == "__main__":
    main()
