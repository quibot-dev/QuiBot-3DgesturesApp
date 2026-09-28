# Qui-Bot H₂O

Robot humanoide educatiu que introdueix la programació seqüencial i la química
a infants de primària. Es mou per un tauler de joc amb línies negres, seguint
seqüències d'ordres, i manipula líquids de colors amb dues xeringues.

Admet tres vies de control:

- **Blocs de colors.** S'hi introdueixen cubs de fusta i el robot en llegeix el
  color i executa l'acció corresponent. És la via original del projecte.
- **Gestos.** Un sensor davant la cara reconeix sis ordres de moviment, amb
  confirmació abans d'executar-les.
- **Control remot per Wi-Fi.** El robot serveix una pàgina web que s'obre des
  del navegador de qualsevol telèfon o ordinador de la mateixa xarxa, sense
  instal·lar res.

## Contingut del repositori

| Directori | Contingut |
|---|---|
| `3D/` | Peces en STL, fitxers de disseny, model del conjunt en FreeCAD, captures amb l'orientació d'impressió i els paràmetres de cada perfil |
| `PCB/` | Projecte KiCad, fitxers de fabricació en Gerber, esquema elèctric i vista 3D |
| `src/` | Programa de control, servidor web i programes de verificació |
| `doc/` | Llista de material i memòria del treball |
| `User_manual/` | Guia d'ús del robot per a tallers i vídeo de demostració |

La **guia de muntatge**, de trenta-set passos, és l'annex A de la memòria.

## Maquinari

El control corre sobre una **Raspberry Pi Zero W** muntada sobre una placa
dissenyada per al projecte. Els motors pas a pas es governen amb controladors
A4988 i els polsos es generen per programari amb `pigpio`.

Els sensors són el TCS34725 per al color dels blocs, el PAJ7620U2 per als
gestos, el VL53L0X per a la distància als recipients i dos TCRT5000 per al
seguiment de línia, llegits a través d'un convertidor ADS1115. Els ulls són
dues matrius de LED WS2811.

## Posada en marxa

Cal Raspberry Pi OS Lite de 32 bits. El procediment complet és a l'annex D de
la memòria; en resum:

**1. Busos i pins.** Afegiu a `/boot/firmware/config.txt`:

```
dtparam=i2c_arm=on
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=0,i2c_gpio_scl=1
gpio=4,5,6=op,dh
```

La tercera línia imposa nivell alt als pins d'habilitació dels controladors des
de l'arrencada i evita que els motors quedin energitzats i brunzint.

**2. Mòdul d'accés als busos:**

```bash
sudo modprobe i2c-dev
echo i2c-dev | sudo tee -a /etc/modules
```

Reinicieu. `ls /dev/i2c-*` ha de mostrar els dos busos.

**3. Entorn de Python:**

```bash
python3 -m venv ~/.venv
~/.venv/bin/pip install pigpio smbus2 flask rpi_ws281x \
    adafruit-circuitpython-ads1x15 \
    adafruit-circuitpython-tcs34725 \
    adafruit-circuitpython-vl53l0x \
    adafruit-extended-bus
```

**4. Dimoni de control dels pins:**

```bash
sudo systemctl enable --now pigpiod
```

**5. Execució:**

```bash
~/.venv/bin/python3 quibot.py
```

El programa escriu a la sortida l'adreça del control remot, que escolta al
port 8000. Per arrencar-lo automàticament amb la màquina, l'annex D descriu el
servei de systemd corresponent.

Els programes de `src/tests/` permeten comprovar els motors i cada sensor per
separat, cosa que convé fer amb la placa encara accessible.

## Procedència i llicències

L'estructura mecànica i el disseny original del robot provenen del projecte
[Qui-Bot H₂O de La Codornella](https://github.com/lacodornella/QuiBot), d'on
surten també les captures d'orientació d'impressió que es reprodueixen aquí.
Dues peces s'han redissenyat i la placa de control és obra de l'equip del
projecte. El control per gestos, el control remot i la documentació són feina
d'aquest treball.

| Element | Llicència |
|---|---|
| Maquinari: peces 3D, PCB i Gerbers | CERN-OHL-S v2 |
| Programari: codi de control | GPL v3 |
| Documentació: guia de muntatge, BOM i manual | CC BY-SA 4.0 |

Totes tres són recíproques: permeten usar, modificar i redistribuir el
material amb la condició que les obres derivades es publiquin en les mateixes
condicions.
