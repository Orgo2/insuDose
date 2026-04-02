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
  - Aktualny PDM clock je 3.0 MHz (PLLSAI1=48 MHz, MCKDIV=8, `SAI_AUDIO_FREQUENCY_96K`). Frame rate = 93.75 kHz.
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
- SAI PDM config: `MicPairsNbr=2, SlotNumber=4, DataSize=8, FrameLength=32, SlotActive=SAI_SLOTACTIVE_2, AudioFrequency=96K, MonoStereoMode=STEREO`.
- S `DataSize=8` a `DMA_PDATAALIGN_WORD`: kazdy 8-bit slot produkuje samostatne 32-bit DMA slovo (8 bitov dat v [7:0], zvysok je zero-padding). Preto s `SAI_SLOTACTIVE_2` (len slot 2 = pair2-L) ide do DMA len 1 word per frame a mic data su v byte 0 `(raw_word & 0xFF)`.
- DOLEZITE: `SlotActive=0xFFFF` s extraciou byte 2 NEFUNGUJE, lebo kazdy slot generuje separatny 32-bit DMA word — byte 2 by bol padding (0x00), nie mic data. Toto bol potvrdeny bug (peak=32768).
- `MonoStereoMode=STEREO` je nutny, lebo `MONO` (CR1.MONO=1) nuti SAI prijmat len slot 0, cim by sa obisiel `SlotActive` a dostal D1 odpad.
- PDM → PCM: 5. rad CIC decimacny filter (CIC5) s R=16:
  - Integrator kaskada (5 stupnov) bezi na PDM bit rate (8 bitov/frame × 93.75k = 750k bits/s).
  - Comb kaskada (5 stupnov) bezi na decimovanom vystupe.
  - Vystupny PCM sample rate = 750k / 16 = 46.875 kHz.
  - CIC5 gain = R^N = 16^5 = 1048576 (21 bitov). Vystup sa >> 5 na signed 16-bit range.
  - Integrators pouzivaju `uint32_t` pre korektny modularny wrap pri dlhych capture.
  - Integrator aj comb loop su genericke (`for k=1..ORDER`), nie hardcoded na 4 stupne.
- DC offset kompenzacia:
  - Automaticke DC-block pristupy (EMA, one-shot cal z prvych vzoriek) boli ODMIETNUTE uzivatelom — vsetky zhorsovali vysledky.
  - Namiesto toho: manualna kalibracia pozadia cez Enter tlacidlo.
  - V `mic_pdm.c` existuje `s_bg_offset` (int32_t), ktory sa odcitava od kazdeho CIC vystupu pred konverziou na int16.
  - Verejne API: `mic_pdm_set_bg_offset(int32_t)` a `mic_pdm_get_bg_offset(void)`.
  - Novy field `avg_signed` (int32_t) v `mic_pdm_result_t` — priemer CIC vystupu so znamienkom (pred odcitanim bg_offset), pouziva sa ako kalibracna hodnota.
- Enter-button kalibracia pozadia (state machine v `app_runtime.c`):
  - Stlacenie Enter spusti kalibracny cyklus, ak je mic idle a nie je aktivna USB session ani dose capture.
  - `MIC_CAL_REQUESTED`: bg_offset sa vynuluje, spusti sa mic capture.
  - `MIC_CAL_WARMUP` (1000 ms): capture bezi, ale data sa zahadzuju (warmup mikrofonu + ustálenie CIC).
  - `MIC_CAL_MEASURING` (1000 ms): novy capture, ktoreho `avg_signed` sa pouzije ako kalibracna hodnota.
  - `MIC_CAL_DONE`: `take_result()` preberie vysledok, `mic_pdm_set_bg_offset(avg_signed)` sa zavola, stav sa vrati na IDLE.
  - Warmup capture sa force-stopne a result sa zahodí; az druhy capture sa pouzije na kalibraciu.
- Display pri validnom mic vysledku ukazuje `P<peak> A<avg> C<offset>`:
  - `P` = peak_abs (16-bit PCM, absolutna hodnota)
  - `A` = avg_abs (16-bit PCM, absolutna hodnota)
  - `C` = aktualny bg_offset (0 = nekalibrované, nenulove = po Enter kalibrácii)
- ZNAMY BUG (2026-04-02): peak ~9000, avg ~2000 v tichu. Kalibracne hodnoty (C=35-160) su realisticke ako pozadie, ale peak/avg su vysoko nad nimi. Niekde je este chyba, ktora zhorsuje peak a avg — treba hladat.
- IM67D120 ma ~25 ms wakeup time po zapnuti CK2. Prvych 2400 DMA slov (~25.6 ms) sa preskoci (CIC nedostane tieto bity, transport metriky sa stale zbieraju).
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
- **BUG: peak ~9000 / avg ~2000 v tichu** — kalibracne hodnoty (C=35-160) sedia, ale peak a avg su anomalne vysoke. Treba najst pricinu (mozne: zly bit extraction, CIC overflow, wakeup skip nedostatocny, interference z D1/PA10, atd.).
- Dose flow: preklopit aktualny runtime na finalny cielovy flow `0 -> 1 -> 0`, aby start/stop mic sedel s mechanikou pera a logovanie prislo az po potvrdeni uzivatelom.
- Dose flow: doplnit stav, kde sa po spracovani vysledku zobrazi namerana davka a caka sa na potvrdenie uzivatelom pred zapisom do logu.
- Mic: overit spravanie pri tichu, hovore a pri zdroji hluku typu PC ventilator.
- Mic: overit, ci 3 MHz PDM clock a CIC4 PCM dava rozumne cisla pre ticho aj zvuk.
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

## Kontrolny zoznam pred dalsou zmenou
- Overit, ci zmena patri do aktualnej temy. Ak nie, len ju zapisat do TODO.
- Navrhnut dalsi krok a pockat na suhlas.
- Drzat riesenie co najjednoduchsie.
- Uprednostnit HAL.
- Po schvalenej zmene urobit full build.
