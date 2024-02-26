# Introduction
This is a simple robot that can balance itself on two wheels using a PID
controller. The robot has the following components:

**No guides on balancing robots were used to make this balancing robot!**

# Components
* [Arduino Pro Mini (ATmega328P, 8Mhz, 3.3V)](https://www.amazon.com/HiLetgo-ATmega328P-Oscillator-Compatible-ATmega128/dp/B07RS911JD/ref=sr_1_3?crid=2N14IKG33WL1A&dib=eyJ2IjoiMSJ9.3cETzf54YDlehGHQvoVDiVIiaq1bia3T7lTLmCs7mIcSlOa0pWSL9nXmIEUP8gyyo6spETrA5HDImmJux9nAyGKNxosgedaMHdG7-1u3t_bkqQf1-7y6-f_s4SHTNdhGFsOmJBmApuRD0T7G_YNa-p-C_cqFzspjWikgrqU6B1qU3Fgut7wr3E-5evWR2WtKvV8gRjU8zD4TPHOGfYbysRZ8oLB_Al4iifvydBplh98.WnHgBaJiBwQoLxha3gJAJ-c6l6i3cqrWoPRuc3sGZS8&dib_tag=se&keywords=arduino+pro+mini+3.3v&qid=1708925395&sprefix=arduino+pro+mini+3.3v%2Caps%2C97&sr=8-3)
* [MPU6050 Gyroscope & Accelerometer](https://www.amazon.com/HiLetgo-MPU-6050-Accelerometer-Gyroscope-Converter/dp/B00LP25V1A/ref=sr_1_3?dib=eyJ2IjoiMSJ9.nQ-HfKOFyZoszrV3cxLK6tLh71T4Dx8jkRlVGhGj_VzjnSkvuXzmbbgUXBfehuXg3dih42B-7O1e6pJ_wmbfq76REDHKtJKewAQCtpUypxHodrL-S_GwIp18RxsDtGfIcqfywNxuynlRYx6n1pP_PK4IOHcAe4MN5HbExyxhNhFgCpgvBUFmUPf0Cv66A55IlmCTEX_MMONaZ2Gx4kgC6iC0wl3dfgpNBaHqXvDAwOw.nf-8SXZoMJ1_ZYF1pTAoX2Mb4sRbFjZRSehs2KHyZfI&dib_tag=se&keywords=mpu6050&qid=1708925361&sr=8-3)
* [DRV8833 Motor Driver](https://www.amazon.com/KOOBOOK-DRV8833-Module-Bridge-Controller/dp/B07S4FVY9M/ref=sr_1_4?crid=38J93BG8Z65YF&dib=eyJ2IjoiMSJ9.dp6vuxGaxaC_fGaKSoR8TQxSFrwQAQhvq51mfcB7K3f16QGxkBICIWue9JGpP9x2PG9Ylr2obsoKdojVFgRGOPPaZehAQc3bid1lH10X-Agk6QnV20Gb9Mm67Y-gOxO7u0WPkDjBzK4_lc9xsd787o1wXwoFeM9-P5tY_eYIsFJByRuSshUL3en3VpWLiQHc.R4_3ql7e0FcPt8D_EuwfK8fC8616P_uY3xZO8QcoLlk&dib_tag=se&keywords=drv8833&qid=1708925335&s=hi&sprefix=drv8833%2Ctools%2C90&sr=1-4)
* [NRF24L01+ Transceiver](https://www.amazon.com/HiLetgo-NRF24L01-Wireless-Transceiver-Module/dp/B00LX47OCY/ref=sr_1_3?crid=2C2DYI2LY0S2N&dib=eyJ2IjoiMSJ9.QUJu9P7emtcMKxUO2DrtdXt4uXEZhvwWqaOctTjonANpBEdCc8Frk0AAZL3Thdsn1h_z7JQ50CiF6QJPXY-HlWeG-CybzILcOkQpC3_ur_S4GOGJNfdTUCbRX48pDtSRhRSSphxfTgqjYyTOC1gPcuSh_08gnAm4LwiimAWdbI_AgsYunSaxhgxYPo8K31SAbLbcF20GhNW7JSxsl65IpjMek_pVFvvks12cqEnUNW6-C66VmsuhM5wQEF-anpGFPy6AvZO0VPllnANotgqOO_FD-qyNZTyvxrMpZvRatIc.25kXNKWOcxNgYiAKsMev3QXyfP0JlmtocFIKHd14Fmg&dib_tag=se&keywords=nrf24l01%2B&qid=1708926122&s=electronics&sprefix=nrf24l01%2B%2Celectronics%2C88&sr=1-3)
* [2x 3-12V DC motors](https://www.amazon.com/gp/product/B07Q2QZQ7Y)
* [3.7V 800mAh LiPo battery](https://www.amazon.com/gp/product/B08GZ3VZ82/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
* [Battery Connectors](https://www.amazon.com/gp/product/B07YQY9V6F/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
* [TPU Filament](https://www.amazon.com/gp/product/B01M9AXXZD/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
* [PLA Filament](https://www.amazon.com/gp/product/B07PGZNM34/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
* [Breadboard](https://www.amazon.com/gp/product/B07LFD4LT6/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
* [Breadboarding Wires](https://www.amazon.com/gp/product/B08YRGVYPV/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)

The above components are just what I picked, feel free to substitute things in
and out. The code is designed specifically for the MPU6050 though.

# CAD
Link to CAD files on OnShape: [Link](https://cad.onshape.com/documents/7b9b6fda841dd45178915373/w/32f2686a3b4c95e17893e537/e/a7000d906e6f19a225cac0ee?renderMode=0&uiState=65dc20009a3c5b36286aaa91)
I've also exported the files from OnShape as .sldwrks ans .stl files for
convenience, you can find them in the 'CAD' directory. These are all the same
files.

# Libraries
The external libraries used in this project are:
* [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
* [RF24](https://www.arduino.cc/reference/en/libraries/rf24/) 

Install these libraries from inside your Arduino IDE.

# How Do I Build This?

You should know, to build this you'll need access to a 3D printer and soldering
equipment.

1. Download the .stl files from OnShape or grab them from the 'CAD' directory. The 'main_body' should be printed with PLA, and the wheels should be printed with TPU.
2. Assemble the robot's breadboard. Here's a picture of my breadboard below:
![breadboard](pics/top.png)
One day, I will create a circuit diagram in KiCad, but this circuit is very
simple so I hope the pictures is enough.
 * I soldered pins onto the Arduino
 Pro Mini, the SCL and SDA pins from the MPU6050 to the A4 and A5 pins on the
 Arduino, and wires onto the motor leads.
3. Install the required external libraries.
4. Upload the code to the Arduino, and you're done! You might need to adjust the
PID gains.

# Helpful Resources
* https://softwareengineering.stackexchange.com/questions/186124/programming-pid-loops-in-c
