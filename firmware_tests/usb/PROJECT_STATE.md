# InsuDose Firmware State

## Produkt
- Elektronicke inzulinove pero s USB MSC logovanim, e-paper displayom a snimanim davky.
- USB rezim sluzi na pristup k logom a servisne/debug pouzitie.

## HW mapa
- `PA2` `Dose_Pin`: mechanika davky, logika udalosti je `0 -> 1 -> 0`
- `PA0` `Enter_Pin`: tlacidlo
- `PA3` `Temp_Alert_Pin`: TMP102 alert, zdielany konflikt s PDM clock vetvou
- `PA4` `Temp_PWR_Pin`: napajanie TMP102, da sa vypnut pocas merania mikrofonom
- `PA8`: planovany `SAI1_PDM_CK2` pre mikrofon IM67D120
- `PA9`: planovany `SAI1_PDM_DI2` pre mikrofon IM67D120
- `PA10` `Ch_chg_Pin`: charger status input, ma byt citany len ked treba
- `PE4` `Power_Detect_Pin`: HW detect externeho napajania, nie spolahlivy indikator zivej USB session
- Display: `D_busy`, `D_rst`, `D_dc`, `D_cs`, `D_clk`, `D_mosi`

## Aktualna funkcionalita
- USB MSC funguje bez zavislosti na klamlivom `PE4`
- Logovanie po `Dose_Pin` sekvencii `0 -> 1 -> 0` ma fungovat aj ked mic bring-up zlyha; mic debug nesmie blokovat log ani display
- E-paper display funguje, je neblokujuci a pocas cakania na `BUSY` vie pustit MCU do `STOP2`
- Display po davke zobrazuje davku, cas od poslednej davky, bateriu a teplotu
- Pri USB zobrazuje samostatnu bateriovu/status obrazovku
- `IM67D120` bring-up modul je znovu napojeny len pasivne: `dose` logika ostava samostatna, mic bezi iba ako sidecar debug bez vplyvu na zapis logu
- Low-level mic driver je uz zredukovany na samotny mikrofon na `PA8/PA9` bez dotyku `PA3/PA10`
- `Mic_PWR` (`PA7`) je docasne drzaný zapnuty od bootu, aby sa vylucil problem napajania mikrofónu

## Stabilne poziadavky
- `Display_EPD_W21*` driver brat ako svaty, menit len ked je na to explicitny dovod
- Po kazdej code zmene robit realny full build
- Ked nie je USB session aktivna, zariadenie musi normalne logovat a bezat na bateriu
- Display ma mat nizku spotrebu a co najmenej zatazovat MCU
- `Insudose` mic/SAI kod bol len referencia, nikdy nebol odskusany

## Aktualny problem
- Po integracii mic bring-upu treba overit 3 veci:
- `dose` udalost musi okamzite zapisat log a obnovit display, aj ked mic capture este bezi alebo zlyha
- display sa musi po strate USB session okamzite vratit z USB obrazovky spat do baterioveho modu
- mic debug data musia byt viditelne bez rozbitia hlavneho layoutu
- Az ked toto sedi, overit na HW, ci `SAI1 PDM` na `PA8/PA9` naozaj zbiera zive data a ci su debug hodnoty realisticke

## Dalsi krok
- Otestovat po poslednom fixe:
  - `dose 0 -> 1 -> 0` zasa okamzite loguje a prekresli display
  - po odpojeni USB sa display vrati z USB obrazovky bez potreby replug
  - mic debug je viditelny aspon ako `MIC ERR/...` alebo realne cisla
- Az potom pokracovat v ladeni samotneho PDM spracovania

## Poznamky pre dalsi chat
- `.ioc` dnes SAI negeneruje
- `HAL_SAI` v aktualnom CubeWB baliku lokalne chyba, pravdepodobne pojdeme cez CMSIS/LL/register level
- `RCC` uz ma pripravene `PLLSAI1` a `SAI1Freq = 48 MHz`
- TMP102 ma byt pocas mic merania vypnuty cez `Temp_PWR`
- `Ch_chg_Pin` ma byt pocas mic merania neaktivny, citat ho len pri potrebe
- ADC clock bol presunuty z `PLLSAI1` na `SYSCLK`, aby `PLLSAI1` mohol vlastnit `SAI1`
- Mic capture zatial pouziva aj `PA3/PA10` v `AF3_SAI1` pocas merania a po stop vracia piny spat
- Mic `PA7` napajanie treba zapinat okamzite pri starte capture, bez umeleho extra delay
- Mic start sa ma robit mimo EXTI IRQ; v IRQ sa ma len oznacit pending event a realny start spustit az v hlavnom ticku
- Pri ladeni mikrofónu najprv nesmie byt rozbita baza: `dose` musi logovat a display/USB sa musia prepinat bez regresii
- Aktualny bezpecny postup: mic moze zbierat debug data na pozadi, ale nesmie rozhodovat o tom, ci sa zapise log alebo prekresli display
