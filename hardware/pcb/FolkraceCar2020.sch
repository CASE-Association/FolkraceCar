EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Folkrace Car 2020"
Date "2020-03-18"
Rev "1.0"
Comp "Noel Danielsson"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_02x20_Odd_Even J2
U 1 1 5E74EEE3
P 3050 5300
F 0 "J2" H 3100 6417 50  0000 C CNN
F 1 "Jetson Nano Header" H 3100 6326 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x20_P2.54mm_Vertical" H 3050 5300 50  0001 C CNN
F 3 "https://www.jetsonhacks.com/nvidia-jetson-nano-j41-header-pinout/" H 3050 5300 50  0001 C CNN
	1    3050 5300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0101
U 1 1 5E75497F
P 4350 4200
F 0 "#PWR0101" H 4350 4050 50  0001 C CNN
F 1 "+5V" H 4365 4373 50  0000 C CNN
F 2 "" H 4350 4200 50  0001 C CNN
F 3 "" H 4350 4200 50  0001 C CNN
	1    4350 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 4400 4350 4400
Wire Wire Line
	4350 4400 4350 4200
Wire Wire Line
	3350 4500 4350 4500
Wire Wire Line
	4350 4500 4350 4400
Connection ~ 4350 4400
$Comp
L power:+3V3 #PWR0102
U 1 1 5E7554D2
P 1950 4250
F 0 "#PWR0102" H 1950 4100 50  0001 C CNN
F 1 "+3V3" H 1965 4423 50  0000 C CNN
F 2 "" H 1950 4250 50  0001 C CNN
F 3 "" H 1950 4250 50  0001 C CNN
	1    1950 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 4400 1950 4400
Wire Wire Line
	1950 4400 1950 4250
Text Label 2850 4500 2    50   ~ 0
I2C_2SDA
Text Label 2850 4600 2    50   ~ 0
I2C_2_SCL
Text Label 2850 4700 2    50   ~ 0
AUDIO_MCLK
Text Label 2850 4900 2    50   ~ 0
UART_2_RTS
Text Label 2850 5000 2    50   ~ 0
SPI_2_SCK
Text Label 2850 5100 2    50   ~ 0
LCD_TE
Text Label 2850 5300 2    50   ~ 0
SPI_1_MOSI
Text Label 2850 5400 2    50   ~ 0
SPI_1_MISO
Text Label 2850 5500 2    50   ~ 0
SPI_1_SCK
Text Label 2850 5600 2    50   ~ 0
GND
Text Label 2850 5700 2    50   ~ 0
I2C_1_SDA
Text Label 2850 5800 2    50   ~ 0
CAM_AF_EN
Text Label 2850 5900 2    50   ~ 0
GPIO_PZ0
Text Label 2850 6000 2    50   ~ 0
GPIO_PE6
Text Label 2850 6100 2    50   ~ 0
I2S_4_LRCK
Text Label 2850 6200 2    50   ~ 0
SPI_2_MOSI
Text Label 2850 6300 2    50   ~ 0
GND
Text Label 3350 4600 0    50   ~ 0
GND
Text Label 3350 4700 0    50   ~ 0
UART_2_TX
Text Label 3350 4800 0    50   ~ 0
UART_2_RX
Text Label 3350 4900 0    50   ~ 0
I2S_4_SCLK
Text Label 3350 5100 0    50   ~ 0
SPI_2_CS1
Text Label 3350 5200 0    50   ~ 0
SPI_2_CS0
Text Label 3350 5300 0    50   ~ 0
GND
Text Label 3350 5400 0    50   ~ 0
SPI_2_MISO
Text Label 3350 5500 0    50   ~ 0
SPI_1_CS0
Text Label 3350 5600 0    50   ~ 0
SPI_1_CS1
Text Label 3350 5700 0    50   ~ 0
I2C_1_SCL
Text Label 3350 5800 0    50   ~ 0
GND
Text Label 3350 5900 0    50   ~ 0
LCD_BL_PWM
Text Label 3350 6100 0    50   ~ 0
UART_2_CTS
Text Label 3350 6200 0    50   ~ 0
I2S_4_SDIN
Text Label 3350 6300 0    50   ~ 0
I2S_4_SDOUT
$Comp
L power:GND #PWR0103
U 1 1 5E75D2C6
P 1950 6500
F 0 "#PWR0103" H 1950 6250 50  0001 C CNN
F 1 "GND" H 1955 6327 50  0000 C CNN
F 2 "" H 1950 6500 50  0001 C CNN
F 3 "" H 1950 6500 50  0001 C CNN
	1    1950 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 4800 1950 4800
Wire Wire Line
	1950 4800 1950 5600
$Comp
L power:+3V3 #PWR0104
U 1 1 5E7675B3
P 2200 5100
F 0 "#PWR0104" H 2200 4950 50  0001 C CNN
F 1 "+3V3" H 2215 5273 50  0000 C CNN
F 2 "" H 2200 5100 50  0001 C CNN
F 3 "" H 2200 5100 50  0001 C CNN
	1    2200 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 5200 2200 5200
Wire Wire Line
	2200 5200 2200 5100
Text Label 3350 5000 0    50   ~ 0
GND
Text Label 3350 6000 0    50   ~ 0
GND
Wire Wire Line
	2850 6300 1950 6300
Connection ~ 1950 6300
Wire Wire Line
	1950 6300 1950 6500
Wire Wire Line
	2850 5600 1950 5600
Connection ~ 1950 5600
Wire Wire Line
	1950 5600 1950 6300
$Comp
L power:GND #PWR0105
U 1 1 5E77B169
P 4350 6500
F 0 "#PWR0105" H 4350 6250 50  0001 C CNN
F 1 "GND" H 4355 6327 50  0000 C CNN
F 2 "" H 4350 6500 50  0001 C CNN
F 3 "" H 4350 6500 50  0001 C CNN
	1    4350 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 4600 4350 4600
Wire Wire Line
	4350 4600 4350 5000
Wire Wire Line
	3350 5000 4350 5000
Connection ~ 4350 5000
Wire Wire Line
	4350 5000 4350 5300
Wire Wire Line
	3350 5300 4350 5300
Connection ~ 4350 5300
Wire Wire Line
	4350 5300 4350 5800
Wire Wire Line
	3350 5800 4350 5800
Connection ~ 4350 5800
Wire Wire Line
	4350 5800 4350 6000
Wire Wire Line
	3350 6000 4350 6000
Connection ~ 4350 6000
Wire Wire Line
	4350 6000 4350 6500
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 5E780E66
P 1950 1850
F 0 "J1" H 2058 2031 50  0000 C CNN
F 1 "Conn_01x02_Male" H 2058 1940 50  0000 C CNN
F 2 "" H 1950 1850 50  0001 C CNN
F 3 "~" H 1950 1850 50  0001 C CNN
	1    1950 1850
	1    0    0    -1  
$EndComp
Text Notes 1700 1500 0    50   ~ 0
Battery Connector
$Comp
L power:GND #PWR0106
U 1 1 5E78A624
P 2350 2200
F 0 "#PWR0106" H 2350 1950 50  0001 C CNN
F 1 "GND" H 2355 2027 50  0000 C CNN
F 2 "" H 2350 2200 50  0001 C CNN
F 3 "" H 2350 2200 50  0001 C CNN
	1    2350 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 1950 2350 1950
Wire Wire Line
	2350 1950 2350 2200
$Comp
L power:+BATT #PWR0107
U 1 1 5E78B717
P 2650 1650
F 0 "#PWR0107" H 2650 1500 50  0001 C CNN
F 1 "+BATT" H 2665 1823 50  0000 C CNN
F 2 "" H 2650 1650 50  0001 C CNN
F 3 "" H 2650 1650 50  0001 C CNN
	1    2650 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 1850 2650 1850
Wire Wire Line
	2650 1850 2650 1650
NoConn ~ 6350 2050
NoConn ~ 6350 2150
NoConn ~ 6350 1750
$Comp
L DCDC:OKL-T_6-W12N-C U2
U 1 1 5E77EE54
P 5300 1850
F 0 "U2" H 5950 2115 50  0000 C CNN
F 1 "OKL-T_6-W12N-C" H 5950 2024 50  0000 C CNN
F 2 "OKL2T6W12PC" H 6450 1950 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/OKL-T_6-W12N-C.pdf" H 6450 1850 50  0001 L CNN
F 4 "MURATA POWER SOLUTIONS - OKL-T/6-W12N-C - DC/DC CONVERTER, 30W, ADJ OUTPUT" H 6450 1750 50  0001 L CNN "Description"
F 5 "7.2" H 6450 1650 50  0001 L CNN "Height"
F 6 "580-OKL-T/6-W12N-C" H 6450 1550 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=580-OKL-T%2F6-W12N-C" H 6450 1450 50  0001 L CNN "Mouser Price/Stock"
F 8 "Murata Electronics" H 6450 1350 50  0001 L CNN "Manufacturer_Name"
F 9 "OKL-T/6-W12N-C" H 6450 1250 50  0001 L CNN "Manufacturer_Part_Number"
	1    5300 1850
	1    0    0    -1  
$EndComp
$Comp
L DCDC:TPS25940x-Q1 U1
U 1 1 5E795B06
P 3750 2450
F 0 "U1" H 3750 3315 50  0000 C CNN
F 1 "TPS25940x-Q1" H 3750 3224 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-20-1EP_3x4mm_P0.5mm_EP1.65x2.65mm_ThermalVias" H 3750 3450 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps25940-q1.pdf" H 4000 2100 50  0001 C CNN
	1    3750 2450
	1    0    0    -1  
$EndComp
Connection ~ 2650 1850
Wire Wire Line
	3200 1950 3050 1950
Wire Wire Line
	3050 1950 3050 1850
Wire Wire Line
	3200 1850 3050 1850
Wire Wire Line
	3050 1850 2650 1850
Connection ~ 3050 1850
Wire Wire Line
	3200 2050 3050 2050
Wire Wire Line
	3050 2050 3050 1950
Connection ~ 3050 1950
Wire Wire Line
	3200 2150 3050 2150
Wire Wire Line
	3050 2150 3050 2050
Connection ~ 3050 2050
Wire Wire Line
	3200 2250 3050 2250
Wire Wire Line
	3050 2250 3050 2150
Connection ~ 3050 2150
$EndSCHEMATC
