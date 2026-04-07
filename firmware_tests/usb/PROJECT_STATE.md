# InsuDose Firmware State

## Intro pre dalsi chat / bota
- Tento subor je externa pamat projektu. Ma rychlo vysvetlit, co je to za zariadenie, co uz funguje, co je rozbite a na com sa prave robi.
- Ma byt prehladny, strucny a aktualny. Ked sa nieco zmeni, oprav to sem a nenechavaj stare tvrdenia.
- Ma byt pouzitelny aj pre bota, ktory projekt nikdy predtym nevidel.

## Pracovne pravidla
- Nerobit zmeny v kode automaticky. Najprv navrhnut dalsi krok a pockat na suhlas pouzivatela.
- Kod drzat co najjednoduchsi a najprehladnejsi.
- Funkcionalitu jedneho zariadenia nestrihat do viacerych suborov bez dovodu. Specializovany kod patri do drivera, `main.c` ma len inicializovat a volat hotove funkcie.
- Nevytvarat medzivrstvy a "medzidrivre", ak na to nie je jasny technicky dovod.
- Uprednostnovat HAL riesenie. Ak je treba ist na register/LL/low-level kod, najprv si vypytat povolenie.
- Nevymyslat si data. Ked sa nieco neda potvrdit, povedat `neviem` alebo `neda sa potvrdit`.
- Tvrdenia a zasahy overovat. Odkazovat sa na konkretne subory, riadky, build alebo HW pozorovanie.
- Po kazdej zmene, ktoru pouzivatel schvali, urobit realny full build. Nevyhovarat sa, ze sa neda najst `makefile`; pouzit STM32Cube build.

## Produkt
- FW pre datalogger na inzulinove pero.
- Zariadenie je napajane z `CR2032` / `LIR` gombikovej baterie, preto musi byt co najviac low power.
- Ma e-paper display, USB MSC pristup k logom, meranie teploty, charger status a rozpracovane snimanie davky mikrofonom.
- USB rezim je urceny na pristup k logom, servisny/debug provoz a nabijanie.

## Aktualna architektura
- `main.c` inicializuje Cube/HAL periferie a vola aplikacnu vrstvu.
- Hlavna logika zariadenia je v runtime vrstve.
- Display ma vlastny driver a app vrstvu.
- Charger, RTC, TMP102 a mic maju byt riesene samostatnymi drivermi.
- Mic driver ma byt HAL-first; low-level verzia ma zostat len ako zaloha/reference, nie ako predvolena cesta.
- Stary SPI EPD driver podla nazvu cipoveho drivera nikdy nefungoval kvoli zlemu testovaciemu displayu a je odlozeny ako `.txt`, aby sa sam neaktivoval.

## Cielove spravanie zariadenia
- Pero ma byt vacsinu casu uspate. MCU sa ma budit len na potrebne udalosti.
- `Dose` a `Enter` su EXTI wake zdroje prave preto, aby zariadenie mohlo takmer stale spat.
- Po skonceni potrebnej prace sa ma MCU vratit do low-power rezimu.

## Cielove spravanie: meranie davky
- Pri kludovom stave pera je na `Dose_Pin` logicka `0`, lebo mechanika tlaci pin o GND.
- Ked uzivatel zacne natahovat davku, `Dose_Pin` prejde co najskor z `0` na `1`.
- Na tejto hrane sa ma zobudit MCU, co najrychlejsie zapnut napajanie mikrofónu, spustit mic capture a zacat pocitat tiky/udalosti davky.
- Ked sa piest pri aplikacii potlaci, mechanika znovu pritlaci tlacidlo a `Dose_Pin` padne z `1` na `0`.
- Na tejto hrane sa ma mic capture ukoncit, vypnut napajanie mikrofónu, spracovat vysledok merania a zobrazit vysledok na display.
- Az po zobrazeni vysledku si ma zariadenie vyziadat potvrdenie spravnosti vysledku od uzivatela. Logovanie davky sa ma urobit az po tomto potvrdeni.
- Menu a potvrdenie vysledku sa budu riesit neskor, ale `Dose` logika musi uz teraz ostat kompatibilna s tymto buducim flow.

## HW mapa
- `PA0` `Enter_Pin`: tlacidlo, EXTI wake zdroj.
- `PA2` `Dose_Pin`: mechanika davky, active-low. V klude `0`, pri natahovani davky ide na `1`, po stlaceni piestu pada spat na `0`.
- `PA3` `Temp_Alert_Pin`: TMP102 alert cez EXTI, ma zostat vyhradeny pre teplomer.
- `PA4` `Temp_PWR_Pin`: napajanie TMP102.
- `PA7` `Mic_PWR_Pin`: napajanie mikrofonu.
- `PA8`: `SAI1_CK2` pre PDM mikrofon.
- `PA9`: `SAI1_D2` pre PDM mikrofon.
- `PA10`: zdielany pin. Pri USB sa sprava ako charger status input. Mimo USB sa pocas mic capture pouzit ako `SAI1_D1`. Nekoliduje to, lebo ide len o vstup do MCU a data z `D1` sa nevyuzivaju.
- `SAI1 CKEN1` sa nesmie pouzit, lebo na tej vetve je TMP102 open-collector. Pre mic ma ostat aktivny len `CKEN2`.
- Realny HW setup mikrofónu:
  - Jeden PDM mikrofón na `D2` linke, clock z `CK2`.
  - `L/R` pin mikrofónu je na GND → mikrofón je "left channel" a vysila data platne na **dobeznej (falling) hrane** CK2.
  - D1 linka (`PA10`) je na charger pullupe — nie su na nej realne mic data, treba ich ignorovat.
  - Ziadne ine mikrofony okrem tohto jedneho nie su pripojene.
  - HAL vyzaduje `MicPairsNbr >= 2` aby aktivoval pair 2 (D2); pair 1 (D1) sa aktivuje tiez ale data z nej su odpad.
  - Mic musi snimat frekvenciou aspon 44 kHz (Nyquist pre 15 kHz peak detection).
  - Aktualny PDM clock je ~3.0 MHz (PLLSAI1=12 MHz, AudioFrequency=192K). Frame rate = 187.5 kHz.
  - Povodne bolo AudioFrequency=96K → PDM clock ~1.5 MHz, co davalo len ~51 dB SQNR a klik pera bol pod sum floor.
  - Pre `ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE` v RX mode: CKSTR=0 → SAI realne sampluje na **falling** edge → to je spravne pre L/R=GND mic.
- Ostatne datove vetvy su fake a maju sa ignorovat.
- `PE4` `Power_Detect_Pin`: HW detect externeho napajania. Nie je to spolahlivy indikator zivej USB session; neskor bola dorobena low-power USB detekcia cez alive packet logiku.
- Display: `D_busy`, `D_rst`, `D_dc`, `D_cs`, `D_clk`, `D_mosi`.

## Aktualna funkcionalita loggera
- Pri boote sa inicializuju RTC, TMP102, charger, display a mic driver.
- Ak je USB session aktivna, zariadenie ide do USB rezimu a spristupni ramdisk/log subor.
- Ak USB session aktivna nie je, zariadenie bezi v bateriovom rezime a po necinosti sa vracia do `STOP2`.
- Pred spankom sa ramdisk snapshotuje do backup flash spolu s TMP102 backup metadatami.
- Po zobudeni sa logger moze obnovit z backup flash len ak su data dostupne a bateria dovoluje restore.
- Aktualny zapis davky v kode sa spusta po hrane `1 -> 0`, ked uz je mic capture dopytane zastaveny; buduci ciel je zapisovat az po potvrdeni uzivatelom.
- Log subor sa dnes zapisuje ako CSV riadky `record,timestamp,dose,temp`.
- Timestamp je vo formate `YYYYMMDDHHMM`.
- Dnes sa do logu zapisuje pevna davka `1u`, kym finalne meranie davky este nie je hotove.

## Aktualna funkcionalita: Dose flow
- EXTI na `Dose_Pin` len nastavi pending stavy, nerobi tazku pracu priamo v IRQ.
- V aktualnom kode pri hrane `0 -> 1` nastavi `s_dose_cycle_armed` a pripravi start mic capture.
- Pri hrane `1 -> 0`, ak bol cyklus armed, nastavi pending logovanie davky a pending stop mic capture.
- Display sa pri hrane `0 -> 1` nema prepisovat; mic vysledok sa teraz odklada bokom a na display sa pusta az pri spracovani `1 -> 0`.
- Dose flow ma 2s safety timeout (`APP_DOSE_MIC_TIMEOUT_MS`): ak mic este bezi po 2s od `1 -> 0` hrany, `mic_pdm_force_stop()` ho okamzite finalizuje a dose sa spracuje s diagnostickym mic vysledkom.
- Mic capture ma navyse 5s hard timeout (`MIC_PDM_MAX_CAPTURE_MS`) v `mic_pdm_task()`, ktory force-finalizuje aj bez dose udalosti.
- Toto je aktualne funkcne spravanie v kode, ale este to nie je finalny cielovy flow pera opisany vyssie.
- Dnes sa po dose udalosti zapisuje pevna hodnota davky `1u`; finalne meranie davky este nie je hotove.

## Aktualna funkcionalita: TMP102 / teplota
- TMP102 sa napaja cez `Temp_PWR`.
- Pri povoleni sa senzor napaja, pocka sa na settle time a potom sa nakonfiguruje do continuous conversion, comparator mode, active-low ALERT a conversion rate `0.25 Hz`.
- `PA3` je ALERT EXTI. IRQ len oznaci pending alert a samotne spracovanie sa robi v `tmp102_task()`.
- Ked alarm nie je aktivny, driver drzi poslednu teplotu a obcas robi idle sample.
- Ked je alarm aktivny, driver priebezne pocita dobu prehriatia a uklada maximum teploty aj casove znacky.
- Pri logovani davky sa dnes cita teplota takto:
- Pri finalize dose flow sa pouzije posledna validna cache teploty; nepyta sa uz cerstve blocking citanie priamo v tejto vetve.
- Dočasne vypnutie/zapnutie TMP102 pocas dose/mic cyklu nema zahodit poslednu validnu cache teploty.
- `TMP102_POWER_SETTLE_MS` je znizene na `25 ms`.
- Pri odchode do sleep snapshotu sa TMP102 backup exportuje do logger metadata a TMP102 sa vypne.

## Aktualna funkcionalita: charger / bateria
- Charger pouziva `CEN`, `CHG` a ADC meranie baterie.
- Po boote drzi charger kratko forcene zapnuty, aby STNS01 presiel do spravneho rezimu.
- `charger_task()` bezne nepremeriava bateriu pri kazdom ticku; drzi si periodicky cache a cerstve meranie si vie vynutit cez `charger_force_measure()`.
- Pri prichode VIN sa spravi forceny measure a obnovi sa charger stav.
- `charger_should_sleep()` rozhoduje o tom, ci ma zariadenie prejst do low-power rezimu pri vybitej baterii.
- `charger_should_restore()` rozhoduje, ci je povolene obnovit logger z backup flash.
- USB obrazovka pouziva charger status `CHARGING`, `FULL/CHARGED`, `IDLE` alebo `FAULT/ERR`.

## Aktualna funkcionalita: RTC
- RTC sa pri boote synchronizuje aspon na build datetime, ak este nema validny cas.
- RTC wakeup timer bezi periodicky raz za sekundu a sluzi ako pravidelny wake zdroj.
- RTC poskytuje datum/cas pre logy, display a TMP102 overtemp casove znacky.
- Validita casu sa drzi v RTC backup registroch.

## Aktualna funkcionalita: beeper
- Beeper je jednoduchy PWM vystup cez `LPTIM1`.
- Pouziva sa ako kratka spatna vazba; napr. pri dose pokuse pocas USB session sa spusti kratke pipnutie.
- Beeper tiez ovplyvnuje sleep: ked bezi, MCU neprechadza do `STOP2`.

## Aktualna funkcionalita: Display
- Display je asynchronny a neblokujuci.
- Po boote spravi uvodny white clear a az potom bezne prekreslovanie.
- V USB rezime zobrazuje samostatnu bateriovu/status obrazovku:
- velku ikonu baterie
- percenta
- textovy stav chargeru `CHARGING`, `CHARGED`, `IDLE` alebo `ERR`
- Mimo USB zobrazuje hlavnu obrazovku davky:
- vlavo poslednu davku
- vpravo cas od poslednej davky
- pod ciarou mic debug riadok
- dolu bateriu pri davke a teplotu pri davke
- Cas od poslednej davky sa obnovuje periodicky po minutach.
- V USB rezime sa stav chargeru periodicky kontroluje a pri zmene sa obrazovka prekresli.
- Display sa ma spravat low-power friendly; ked je busy a nie je stop2-safe, MCU ostava len vo `WFI`, inak moze ist do `STOP2`.

## Aktualna funkcionalita: mic
- Momentalne sa riesi mikrofon. Ine temy len zapisovat do TODO, neopravovat bez pokynu.
- Mic driver je teraz HAL verzia v [Core/Src/mic_pdm.c](c:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/usb/Core/Src/mic_pdm.c).
- Povodny low-level mic driver je zaloha v [Core/Src/mic_pdm_lowlevel_backup_2026-03-31.txt](c:/Users/orgo/Documents/Rado/git/insuDose/firmware_tests/usb/Core/Src/mic_pdm_lowlevel_backup_2026-03-31.txt).
- Mic start ma ist mimo EXTI IRQ; v IRQ sa len nastavi pending event.
- Mic debug nesmie blokovat `Dose` flow, display ani low-power spravanie zariadenia.
- Mic capture je zatial len debug sidecar. Nesmie zatial rozhodovat o finalnom logovani davky; to pride az s neskorsim potvrdenim vysledku uzivatelom.
- Ak sa mic capture vobec nerozbehne alebo zlyha jeho start, dose flow sa nesmie zaseknut; davka sa ma aj tak spracovat bez mic vysledku.
- `mic_pdm_force_stop()` okamzite finalizuje capture a spravi vysledok dostupny cez `take_result()`, aj ked capture neprodukoval realne data.
- `MIC_PDM_DEBUG_ALWAYS_POWERED` je aktivne, teda mic napajanie ostava pre debug stale zapnute.
- HAL mic cesta ma dopojeny `SAI1_IRQHandler()` a v `mic_pdm_prepare_sai()` sa SAI config rucne dorovnava.
- HAL mic start ma teraz pred kazdym capture vynuteny `HAL_SAI_DeInit()` → `HAL_SAI_Init()` par, co zarucuje balancovany `MspDeInit → MspInit` cyklus a cistu reinicializaciu SAI1 clock tree, DMA a NVIC.
- `mic_pdm_stop_hw()` uz nevola `HAL_SAI_DeInit()` — DeInit je odlozeny do `mic_pdm_prepare_sai()` prednasledujuceho capture.
- Mic je RX-only. `hdmatx` nesmie byt linknuty na ten isty DMA handle ako `hdmarx`, lebo po `HAL_SAI_DeInit()` to rozbije dalsi capture cyklus.
- SAI PDM config: `MicPairsNbr=2, SlotNumber=4, DataSize=8, FrameLength=32, SlotActive=SAI_SLOTACTIVE_2, AudioFrequency=192K, MonoStereoMode=STEREO`.
- S `DataSize=8` a `DMA_PDATAALIGN_WORD`: kazdy 8-bit slot produkuje samostatne 32-bit DMA slovo (8 bitov dat v [7:0], zvysok je zero-padding). Preto s `SAI_SLOTACTIVE_2` (len slot 2 = pair2-L) ide do DMA len 1 word per frame a mic data su v byte 0 `(raw_word & 0xFF)`.
- DOLEZITE: `SlotActive=0xFFFF` s extraciou byte 2 NEFUNGUJE, lebo kazdy slot generuje separatny 32-bit DMA word — byte 2 by bol padding (0x00), nie mic data. Toto bol potvrdeny bug (peak=32768).
- `MonoStereoMode=STEREO` je nutny, lebo `MONO` (CR1.MONO=1) nuti SAI prijmat len slot 0, cim by sa obisiel `SlotActive` a dostal D1 odpad.
- DMA buffer: 4096 slov (half = 2048, ~10.7 ms pri 192K). Povodne 2048 spôsobovalo DMA overrun pri 31-bin Goertzel skene.
- PDM → PCM: 4. rad CIC decimacny filter (CIC4) s R=32:
  - Integrator kaskada (4 stupne) bezi na PDM bit rate (8 bitov/frame × 187.5k = 1.5M bits/s).
  - Comb kaskada (4 stupne) bezi na decimovanom vystupe.
  - Vystupny PCM sample rate = 1.5M / 32 = 46.875 kHz (rovnaky ako predtym).
  - CIC4 gain = R^N = 32^4 = 1048576 (21 bitov). Vystup sa >> 9 na signed 16-bit range.
  - R=32 + PDM clock 3 MHz dava ~75 dB SQNR (oproti ~51 dB s R=16 a 1.5 MHz clock).
  - CIC5 bol odmietnute: prilis vysoky sum floor (P~8668, A~2075 v tichu), nevyhovujuci pre detekciu kliknutia.
  - Integrators pouzivaju `uint32_t` pre korektny modularny wrap pri dlhych capture.
  - Integrator aj comb loop su genericke (`for k=1..ORDER`), nie hardcoded na 4 stupne.
- DC offset kompenzacia:
  - Automaticke DC-block pristupy (EMA, one-shot cal z prvych vzoriek) boli ODMIETNUTE uzivatelom — vsetky zhorsovali vysledky.
  - Namiesto toho: manualna kalibracia pozadia cez Enter tlacidlo.
  - V `mic_pdm.c` existuje `s_bg_offset` (int32_t), ktory sa odcitava od kazdeho CIC vystupu pred konverziou na int16.
  - Verejne API: `mic_pdm_set_bg_offset(int32_t)` a `mic_pdm_get_bg_offset(void)`.
  - Novy field `avg_signed` (int32_t) v `mic_pdm_result_t` — priemer CIC vystupu so znamienkom (pred odcitanim bg_offset), pouziva sa ako kalibracna hodnota.
- Enter-button kalibracia pozadia + Goertzel prahu (state machine v `app_runtime.c`):
  - Stlacenie Enter spusti kalibracny cyklus, ak je mic idle a nie je aktivna USB session ani dose capture.
  - `MIC_CAL_REQUESTED`: bg_offset sa vynuluje, spusti sa jeden mic capture.
  - `MIC_CAL_WARMUP` (100 ms): capture bezi, ale Goertzel kalibracia este nestartuje. Ciel: nechat sa usadit tlacidlove kliknutie (25 ms wakeup skip je uz v driveri). Jeden capture, bez force-stop.
  - `MIC_CAL_MEASURING` (15 ms): `mic_pdm_start_goertzel_cal()` sa zavola, Goertzel sbiera ~11 binov = mean + 3σ pre prahovu hodnotu. Po 15 ms `mic_pdm_request_stop()`.
  - `MIC_CAL_DONE`: `take_result()` preberie vysledok, `mic_pdm_set_bg_offset(avg_signed)` a `mic_pdm_finish_goertzel_cal()` sa zavola (threshold = mean + 3σ). Display sa okamzite prebudi cez `app_display_note_mic_debug()` a ukaze vysledok.
  - Cely kalibracny priebeh je jeden mic capture (nie dva): warmup a meranie prebehnu v ramci toho isteho DMA streamu.
- Display pri validnom mic vysledku:
  - Spike detektor (aktualny): `R<ratio> P<peak_rms> B<bg_rms> <windows>`.
  - Goertzel scan (ak by sa zapol): `K<peak_k> P<peak_mag> T<target_mag> <bins>`.
  - Goertzel single-bin (ak by sa zapol): `G<aktivne>/<celkove> T<prah>`.
  - API `app_display_note_mic_debug(result, battery_mv)` — bez `is_cal` parametra.
- Display sa nevolá pocas aktivneho mic capture (Dose_Pin=1, tlacidlo NIE je stlacene = mic meria). Ked je Dose_Pin=0 (tlacidlo stlacene piestu), mic sa zastavi a display moze kreslit. Bitbangovy SPI display driver blokuje MCU na desiatky ms pocas prenosu framebufferu a sposoboval stratu DMA half-complete callbackov, co skracovalo pocet spracovanych Goertzel binov. Riesenie: `app_display_task()` sa v `app_runtime_tick()` preskoci ak `app_is_mic_capture_active()` vracia true.
- Goertzel detektor kliknutia (~15.4 kHz, `mic_pdm.c`):
  - N=64 vzoriek/bin, k=21, frekvencia = 21 × 46875 / 64 ≈ 15381 Hz, koeficient = 2·cos(2π·21/64) = −0.9428.
  - Magnitude: sqrt(s1² + s2² − coeff·s1·s2), float aritmetika na Cortex-M4F FPU.
  - Kalibracia: Welfordov online algoritmus pre mean a varianceiu, threshold = mean + 3σ.
  - Pocas dose capture: kazdy bin s mag > threshold sa rata ako `goertzel_bins_active`.
  - Pocet binov × ~1.37 ms = odhadovana dlzka detekovanej udalosti.
  - API: `mic_pdm_start_goertzel_cal()`, `mic_pdm_finish_goertzel_cal()`, `mic_pdm_set/get_goertzel_threshold()`.
  - Aktualne: `GOERTZEL_SCAN_MODE=0` (vypnute), nahradene sirokopasmovym spike detektorom.
- Goertzel frekvenčný sken (historicky, teraz vypnute):
  - Skusili sa 31 paralelnych binov (k=1..31) aj 16 binov (k=2,4,...,32 s krokom 2).
  - 31-bin verzia sposobovala DMA overrun pri -O0 (44 CAL binov namiesto 60).
  - 16-bin + 4096 DMA buffer overrun odstranili (68 CAL binov = korektne).
  - Vysledky: rozdiely dose vs empty boli nahodne (±100–1000), ziadny konzistentny frekvencny vzor.
  - Sporadicky sa objavili velke rozdiely (napr. k=22 +3307, k=12 +3625), ale na rôznych frekvenciach a neopakovatelne — artefakty manipulacie, nie klik.
- Sirokopasmovy spike detektor (aktualny rezim, `SPIKE_DETECT_MODE=1`):
  - Okno 256 vzoriek (~5.46 ms pri 46.875 kHz), pocita energiu (sum-of-squares) kazdého okna.
  - CAL: pocita priemer energie okien → `s_spike_bg` (pozadie).
  - MES: sleduje maximalnu energiu okna (`s_spike_peak`), pocita okna s energiou > 3× a > 5× bg, a pocita lokalny peak count cez `s_spike_peak_count`.
  - DC HP filter: α = 1/512, cutoff ~14.6 Hz — signalove pasmo 23–114 Hz prechádza bez rezu.
  - spikes_3x/5x sa pocitaju z raw `win_e`.
  - Vysledok v `mic_pdm_result_t`: `scan_peak_mag` = peak_rms, `scan_target_mag` = bg_rms, `scan_peak_k` = ratio (capped 255), `goertzel_bins_active` = spikes_3x, `spike_clusters` = peak_count.
  - Display: `R<ratio> P<peak_rms> B<bg_rms> <windows>`.
  - Log format (MES/CAL): `record,timestamp,CAL/MES,total_windows,bg_rms,peak_rms,ratio,spikes_3x,peak_count`.
  - `spike_clusters` field je v `mic_pdm_result_t` (`mic_pdm.h`); `app_runtime.c` MES/CAL logy maju 5 stlpcov (pridany 5. stlpec = peak_count).
  - **Frekvencna analyza signalu (z ACCUM_FFT dat):** signál pera je mechanicka vibracia v pasme 23–114 Hz. Nad ~700 Hz je energia zanedbatelna. FH pasmo (>6.8 kHz) je mrtve.
- **Kontaktny snimac (MEMS cez penovu pasku na pero) — funguje (od 2026-04-06):**
  - IM67D120 prilepeny penovou paskou priamo na pero: ratio 9–22, spikes_3x 13–24 pri dose.
  - Klik pera je dobre viditelny ako energeticka udalost v sirokopasmovom detektore.
  - Problem: POCITANIE jednotlivych klikov je nepresne (viz historia nizsie).
  - EMA vyhladenie energie: `s_spike_smooth_e += 0.25 * (win_e - s_spike_smooth_e)` (α=0.25, cutoff ~7 Hz).
  - Klik = EMA energie stupala nad `SPIKE_DETECT_FACTOR` (3.0)×bg, dosiahla peak, klesla.
  - Po pocitanom peaku sa nastavi `s_spike_post_peak = true`, `s_spike_fell_below = false`.
  - Re-arm vyzaduje DVE podmienky:
    1. EMA musi klestnut pod `SPIKE_REARM_FACTOR` (1.5)×bg → `fell_below = true` (hystereza)
    2. EMA musi potom zacat stupat → re-arm (smerovy guard)
  - Hysterezne pasmo je medzi 1.5×bg a 3×bg. Toto umoznuje re-arm aj pocas sustained activity, zatial co smerovy guard brani wobble re-armom.
  - Stav: `s_spike_smooth_e` (float), `s_spike_post_peak` (bool), `s_spike_fell_below` (bool), `s_spike_was_rising` (bool), `s_spike_prev_smooth` (float), `s_spike_peak_count` (uint32_t).
  - Vysledok ulozen v `s_result.spike_clusters`.
  - **Stav: implementovane, caka na testovanie na HW.**
  - **Lokalny peak detektor klikov + EMA + smerovy decay + hysterezny re-arm (aktualny pristup):**
    - EMA vyhladenie energie: `s_spike_smooth_e += 0.25 * (win_e - s_spike_smooth_e)` (α=0.25, cutoff ~7 Hz).
    - Klik = EMA energie stupala nad `SPIKE_DETECT_FACTOR` (3.0)×bg, dosiahla peak, klesla.
    - Po pocitanom peaku sa nastavi `s_spike_post_peak = true`, `s_spike_fell_below = false`.
    - Re-arm vyzaduje DVE podmienky:
      1. EMA musi klestnut pod `SPIKE_REARM_FACTOR` (1.5)×bg → `fell_below = true` (hystereza)
      2. EMA musi potom zacat stupat → re-arm (smerovy guard)
    - Hysterezne pasmo je medzi 1.5×bg a 3×bg. Toto umoznuje re-arm aj pocas sustained activity, zatial co smerovy guard brani wobble re-armom.
    - Stav: `s_spike_smooth_e` (float), `s_spike_post_peak` (bool), `s_spike_fell_below` (bool), `s_spike_was_rising` (bool), `s_spike_prev_smooth` (float), `s_spike_peak_count` (uint32_t).
    - Vysledok ulozen v `s_result.spike_clusters`.
    - **HW test 2026-04-06:**
      - 4j+4j: 7, 4, 4, 4 (hore/dole)
      - 9j+9j: 5, 3, 1, 5 (hore/dole)
      - Komentár: 4j stále prepočítava (7), 9j podpočítava (1–5), hysteréza 1.5×bg je už dosiahnuteľná, ale stále nie je stabilná.
      - Docasne doplneny debug log obalky `EW*`: po `MES`/`FL`/`FH` sa loguju riadky `EW0`, `EW1`, ... po 32 hodnotach, max 384 okien. Hodnota je `10 * EMA_energy / bg_energy`, cize `15` = re-arm prah a `30` = detect prah. Najblizsi HW test: pozriet, ci EMA medzi realnymi klikmi pada pod 15 a kde vznikaju falosne/preskocene peaky.
      - Build po doplneni `EW*` debug logu: 2026-04-07 full build OK cez CubeIDE `make.exe -C Debug all`. Ostali iba existujuce warningy: nepouzite `s_gtz_s1/s_gtz_s2` pri vypnutom single-bin Goertzel mode a linker warning `usb.elf has a LOAD segment with RWX permissions`.
- **Historia pokusov o pocitanie klikov (pre buduce referencie):**
  - GAP cluster counter, GAP=15 (~21 ms): overcounting. Intra-klik ringing > 15 okien → kazde ozvena pocitana ako novy klik.
  - GAP cluster counter, GAP=35 (~48 ms): undercounting. Pri rychlom tempe je inter-klik pauza < 35 okien → susedne kliky sa zlucuju do jedneho clustra.
  - Onset + refractory, REFRACTORY=50 (~69 ms): undercounting. Energia ostava nad prahom nepretrzite cez viacero klikov → iba 1 onset na cely blok.
  - Koren problemu: kontaktny snimac neukazuje cistocaste per-klik bursts — energia medzi klikmi zostava elevated (vibracne prelievanie cez penovu pasku).
  - Lokalny peak detektor na raw `win_e`, okno 64 vzoriek (1.37 ms): overcounting ~4× (4j→16, 10j→13–24). Pricina: signál pera je vibrácia ~80 Hz, energia `ac²` osciluje na 160 Hz = 4–5 falošných peakov na klik v ramci jedneho okna 732 Hz. Navyse DC HP filter α=1/128 (cutoff ~58 Hz) rezal priamo do signaloveho pasma 23–57 Hz.
  - Lokalny peak detektor + EMA vyhladenie + refractory=12 (5.46ms okno): 4j→13 (+9, pomalé), 10j→11 (+1, rýchle). Konzistencia vyborna (identické páry). Problem: pri pomalych klikoch (~375ms/klik) peró produkuje ~3 oddelene vibracne udalosti na klik (odraz pruziny), kazda ma vlastny EMA peak. Casova refractory nestaci pokryt rozstup medzi odrazmi.
  - Lokalny peak detektor + EMA + hystereza 2×bg: NESTABILNE. 4j→11/9/5/14/4 (obrovska variancia). Pricina: absolutny prah 2×bg sa niekedy prekroci pocas ringingu a niekedy nie — zavisi od presneho tvaru signalu, nie od poctu klikov.
  - Lokalny peak detektor + EMA + smerovy decay tracker (bez fell_below guardu): lepsie nez hystereza, 0j→0 OK, no 4j→2-3 (undercounting), 10j→12-14 (overcounting). Pricina overcountingu: EMA wobble pocas decayu nad 3×bg okamzite re-armuje detektor, kazdy wobble-up-down nad 3×bg = falosny peak.
  - Lokalny peak detektor + EMA + smerovy decay + fell_below@3×bg: MASIVNY UNDERCOUNTING. 4j→1/4, 9j→1/1. Pricina: fell_below vyzadoval EMA < 3×bg = rovnaky prah ako detekcia. EMA pocas sustained klikania nikdy neklesne pod 3×bg (85% okien je nad 3×bg). Re-arm nikdy nenastane.
  - Lokalny peak detektor + EMA + smerovy decay + hysterezny re-arm 1.5×bg (aktualne):
    - re-arm prah znizeny na 1.5×bg. Hysterezne pasmo 1.5×–3×bg umoznuje re-arm aj pocas sustained activity. Smerovy guard brani wobble re-armom.
    - HW test 2026-04-06: 4j+4j: 7/4/4/4, 9j+9j: 5/3/1/5. 4j stále prepočítava (7), 9j podpočítava (1–5), hysteréza 1.5×bg je už dosiahnuteľná, ale stále nie je stabilná.
- IM67D120 ma ~25 ms wakeup time po zapnuti CK2. Prvych 4800 DMA slov (~25.6 ms) sa preskoci (CIC nedostane tieto bity, transport metriky sa stale zbieraju).
- Popri CIC PCM sa zbieraju surove transport sanity metriky z DMA bufferu:
  - pocet raw slov `0x00000000`
  - pocet raw slov `0xFFFFFFFF`
  - minimalny a maximalny `popcount` v raw slove
  - prve a posledne raw slovo capture
- Pri nevalidnom mic vysledku display teraz zobrazuje diagnostiku v tvare `T?S?E??C?N?`:
  - `T` = stage HAL startu
  - `S` = `HAL_SAI_GetState()`
  - `E` = low byte `HAL_SAI_GetError()`
  - `C` = realne videna SAI clock frekvencia v MHz
  - `N` = aktualne DMA `CNDTR`
- Ak sa capture rozbehne a nazbiera aspon nejake raw PDM slova, display pri nevalidnom vysledku preferuje transport sumarnik `T? Z? F? P?-?`:
  - `T` = stage HAL startu
  - `Z` = pocet raw slov `0x00000000`
  - `F` = pocet raw slov `0xFFFFFFFF`
  - `Pmin-max` = minimalny a maximalny `popcount` v raw slove
- Pri validnom vysledku display ukazuje `P<peak> A<avg> C<offset>` (vid vysssie).

## Potvrdene funkcne
- USB MSC funguje.
- Zapisovanie logov funguje v sucasnom flow.
- Display funguje cez bitbang `Display_EPD_W21*` driver, nie cez SPI HAL.
- USB obrazovka a bezna bateriova obrazovka existuju.
- RTC dava cas pre logy a je napojene na periodicky wakeup.
- TMP102 alert je vratene na `PA3` a mic ho uz neprebera.
- Mic transport vrstva je momentalne prehodena na HAL `SAI + DMA`.
- Projekt sa po poslednych zmenach realne prelozi cez `make -C Debug all`.

## Zname technicke pravidla projektu
- `Display_EPD_W21*` brat opatrne. Menit len ked je na to explicitny dovod.
- Ked nie je USB session aktivna, zariadenie musi normalne logovat a bezat na bateriu.
- Display ma mat nizku spotrebu a co najmenej zatazovat MCU.
- Pri ladeni mikrofónu najprv nesmie byt rozbita baza: `dose`, USB, logovanie, display a low-power spravanie musia fungovat bez regresii.

## Otvorene TODO
- Dose flow: preklopit aktualny runtime na finalny cielovy flow `0 -> 1 -> 0`, aby start/stop mic sedel s mechanikou pera a logovanie prislo az po potvrdeni uzivatelom.
- Dose flow: doplnit stav, kde sa po spracovani vysledku zobrazi namerana davka a caka sa na potvrdenie uzivatelom pred zapisom do logu.
- Mic: overit presnost lokalneho peak detektora pre rozne pocty jednotiek (5j, 8j, 9j, 11j) a rozne tempa natahnutia.
- Mic: overit spravanie pri tichu, hovore a pri zdroji hluku typu PC ventilator.
- Mic: overit, ci 3 MHz PDM clock a CIC4 R=32 PCM dava rozumne cisla pre ticho aj zvuk.
- Low power: overit, ze po ukonceni merania davky sa mic napajanie vypne a MCU sa vrati spat do sleep rezimu.
- Bateria/ADC: vycitavanie napatia baterie nie je spolahlive. FW casto drzi staru hodnotu a az po case skokovo prejde na novu.
- Bateria/ADC: pri vypise treba pouzit cerstve meranie, nie stare cache bez overenia.
- Bateria/ADC: percenta baterie sa casto nezhoduju s realne nameranou hodnotou.
- Backup logu: po dlhom behu a vybiti baterie zariadenie zabudlo data.
- Backup logu: po pripojeni USB sa obnovil nulovy subor, backup flash zaloha ramdisk/logu zjavne nefunguje spolahlivo.

## Posledne dolezite rozhodnutia
- Nepouzivany SPI display driver `gdem0097t61` bol odlozeny mimo build ako `.txt`, aby sa po refreshoch sam neaktivoval.
- Mic sa prepol z low-level/register pristupu na HAL `SAI + DMA`.
- `PA3` ostava vyhradne TMP102 alert.
- `PA10` je zamerne zdielany medzi charger status a `SAI1_D1` podla rezimu zariadenia.
- `Dose` hrany su teraz prepnute na fyzicky flow pera `0 -> 1 -> 0`.
- SAI AudioFrequency zmenene z 96K na 192K (PDM clock ~3 MHz, lepsie SQNR).
- CIC R zvysene z 16 na 32 (>>5 → >>9). PCM rate ostava 46.875 kHz.
- DMA buffer zdvojnasobeny z 2048 na 4096 slov (riesenie DMA overrun pri skenovacom rezime).
- Goertzel frekvenčný sken (31-bin aj 16-bin) vyskusany a odlozeny — nepriniesol detekovatelny signal.
- Sirokopasmovy spike detektor implementovany — MEMS mic v pouzdri nevidel klik (ratio ~1.3, spikes=0).
- Kontaktny snimac (MEMS lepeny penovou paskou na pero): klik pera viditelny (ratio 9–22), toto je aktualne pouzivana metoda.
- Log format rozsireny na 5 stlpcov: pridany `peak_count` (= `spike_clusters` z `mic_pdm_result_t`).
- Pocitanie klikov: fell_below prah znizeny z 3×bg na 1.5×bg (SPIKE_REARM_FACTOR). Detekcia ostava na 3×bg (SPIKE_DETECT_FACTOR). Hysterezne pasmo 1.5×–3×bg: re-arm je dosiahnutelny medzi klikmi, ale dostatocne vysoko nad sumom. Smerovy guard zachovany.
- Docasne doplneny `EW*` debug log obalky pre dalsi HW test: `EW` hodnota = `10 * EMA_energy / bg_energy`; `15` je re-arm prah, `30` je detect prah. Ciel je zistit, ci sa EMA medzi klikmi vobec re-armuje a ci falosne peaky vznikaju z odrazov alebo wobble decayu.
- Full build po `EW*` debug zmene bol uspesny 2026-04-07. Build pouzil explicitnu CubeIDE cestu na `make.exe`, lebo `make` nebol v aktualnom `PATH`.
- DC HP filter posunuty z α=1/128 (cutoff ~58 Hz) na α=1/512 (cutoff ~14.6 Hz) — predosla hodnota rezala signalove pasmo 23–57 Hz.
- SPIKE_WIN_SIZE zvacseny z 64 na 256 vzoriek (5.46 ms): menej window evaluacii, stabilnejsia energia, okno uz nie je v konflikte s 160 Hz energetickou oscilaciou.
- EMA vyhladenie `s_spike_smooth_e` (α=0.25) pridane pred peak detekciou: utlmuje sub-klik oscilacie, peak detektor pracuje na hladkom profile.

## Kontrolny zoznam pred dalsou zmenou
- Overit, ci zmena patri do aktualnej temy. Ak nie, len ju zapisat do TODO.
- Navrhnut dalsi krok a pockat na suhlas.
- Drzat riesenie co najjednoduchsie.
- Uprednostnit HAL.
- Po schvalenej zmene urobit full build.
