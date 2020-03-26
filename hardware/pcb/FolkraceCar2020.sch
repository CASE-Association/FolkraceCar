EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Folkrace Car 2020 laZEr"
Date "2020-03-18"
Rev "1.0"
Comp "Noel Danielsson"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:+5V #PWR0101
U 1 1 5E75497F
P 9850 5400
F 0 "#PWR0101" H 9850 5250 50  0001 C CNN
F 1 "+5V" H 9865 5573 50  0000 C CNN
F 2 "" H 9850 5400 50  0001 C CNN
F 3 "" H 9850 5400 50  0001 C CNN
	1    9850 5400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0104
U 1 1 5E7675B3
P 10150 5400
F 0 "#PWR0104" H 10150 5250 50  0001 C CNN
F 1 "+3V3" H 10165 5573 50  0000 C CNN
F 2 "" H 10150 5400 50  0001 C CNN
F 3 "" H 10150 5400 50  0001 C CNN
	1    10150 5400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5E77B169
P 9650 8350
F 0 "#PWR0105" H 9650 8100 50  0001 C CNN
F 1 "GND" H 9655 8177 50  0000 C CNN
F 2 "" H 9650 8350 50  0001 C CNN
F 3 "" H 9650 8350 50  0001 C CNN
	1    9650 8350
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 5E780E66
P 900 1850
F 0 "J1" H 1008 2031 50  0000 C CNN
F 1 "Conn_01x02_Male" H 1008 1940 50  0000 C CNN
F 2 "" H 900 1850 50  0001 C CNN
F 3 "~" H 900 1850 50  0001 C CNN
	1    900  1850
	1    0    0    -1  
$EndComp
Text Notes 650  1500 0    50   ~ 0
Battery Connector
$Comp
L power:GND #PWR0106
U 1 1 5E78A624
P 1300 3500
F 0 "#PWR0106" H 1300 3250 50  0001 C CNN
F 1 "GND" H 1305 3327 50  0000 C CNN
F 2 "" H 1300 3500 50  0001 C CNN
F 3 "" H 1300 3500 50  0001 C CNN
	1    1300 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 1950 1300 1950
Wire Wire Line
	1300 1950 1300 2350
$Comp
L power:+BATT #PWR0107
U 1 1 5E78B717
P 1800 1650
F 0 "#PWR0107" H 1800 1500 50  0001 C CNN
F 1 "+BATT" H 1815 1823 50  0000 C CNN
F 2 "" H 1800 1650 50  0001 C CNN
F 3 "" H 1800 1650 50  0001 C CNN
	1    1800 1650
	1    0    0    -1  
$EndComp
NoConn ~ 7850 2150
NoConn ~ 7850 2250
NoConn ~ 7850 1850
$Comp
L DCDC:OKL-T_6-W12N-C U2
U 1 1 5E77EE54
P 6550 1750
F 0 "U2" H 7200 2015 50  0000 C CNN
F 1 "OKL-T_6-W12N-C" H 7200 1924 50  0000 C CNN
F 2 "DCDC:OKL2T6W12PC" H 7700 1850 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/OKL-T_6-W12N-C.pdf" H 7700 1750 50  0001 L CNN
F 4 "MURATA POWER SOLUTIONS - OKL-T/6-W12N-C - DC/DC CONVERTER, 30W, ADJ OUTPUT" H 7700 1650 50  0001 L CNN "Description"
F 5 "7.2" H 7700 1550 50  0001 L CNN "Height"
F 6 "580-OKL-T/6-W12N-C" H 7700 1450 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=580-OKL-T%2F6-W12N-C" H 7700 1350 50  0001 L CNN "Mouser Price/Stock"
F 8 "Murata Electronics" H 7700 1250 50  0001 L CNN "Manufacturer_Name"
F 9 "OKL-T/6-W12N-C" H 7700 1150 50  0001 L CNN "Manufacturer_Part_Number"
	1    6550 1750
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
Wire Wire Line
	3200 1950 3050 1950
Wire Wire Line
	3050 1950 3050 1850
Wire Wire Line
	3200 1850 3050 1850
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
$Comp
L Analog_ADC:ADS1115IDGS U3
U 1 1 5E77ACD6
P 3150 9800
F 0 "U3" H 2900 10250 50  0000 C CNN
F 1 "ADS1115IDGS" H 3450 9450 50  0000 C CNN
F 2 "Package_SO:TSSOP-10_3x3mm_P0.5mm" H 3150 9300 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/ads1113.pdf" H 3100 8900 50  0001 C CNN
	1    3150 9800
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0108
U 1 1 5E79C44B
P 4550 1650
F 0 "#PWR0108" H 4550 1500 50  0001 C CNN
F 1 "VBUS" H 4565 1823 50  0000 C CNN
F 2 "" H 4550 1650 50  0001 C CNN
F 3 "" H 4550 1650 50  0001 C CNN
	1    4550 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 1850 4400 1850
Wire Wire Line
	4300 1950 4400 1950
Wire Wire Line
	4400 1950 4400 1850
Connection ~ 4400 1850
Wire Wire Line
	4400 1850 4550 1850
Wire Wire Line
	4300 2050 4400 2050
Wire Wire Line
	4400 2050 4400 1950
Connection ~ 4400 1950
Wire Wire Line
	4300 2150 4400 2150
Wire Wire Line
	4400 2150 4400 2050
Connection ~ 4400 2050
Wire Wire Line
	4300 2250 4400 2250
Wire Wire Line
	4400 2250 4400 2150
Connection ~ 4400 2150
Wire Wire Line
	1100 1850 1600 1850
Wire Wire Line
	1800 1650 1800 1850
Connection ~ 1800 1850
Wire Wire Line
	1800 1850 1950 1850
$Comp
L Device:CP C1
U 1 1 5E7BBCD7
P 1600 2100
F 0 "C1" H 1718 2146 50  0000 L CNN
F 1 "CP" H 1718 2055 50  0000 L CNN
F 2 "" H 1638 1950 50  0001 C CNN
F 3 "~" H 1600 2100 50  0001 C CNN
	1    1600 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 2250 1600 2350
Wire Wire Line
	1600 2350 1300 2350
Connection ~ 1300 2350
Wire Wire Line
	1300 2350 1300 3450
Wire Wire Line
	1600 1950 1600 1850
Connection ~ 1600 1850
Wire Wire Line
	1600 1850 1800 1850
Wire Wire Line
	3200 3000 3200 3450
Wire Wire Line
	3200 3450 2950 3450
Connection ~ 1300 3450
Wire Wire Line
	1300 3450 1300 3500
Wire Wire Line
	3750 3100 3750 3450
Wire Wire Line
	3750 3450 3200 3450
Connection ~ 3200 3450
$Comp
L Device:C C2
U 1 1 5E7C9EB8
P 1950 2100
F 0 "C2" H 2065 2146 50  0000 L CNN
F 1 "C" H 2065 2055 50  0000 L CNN
F 2 "" H 1988 1950 50  0001 C CNN
F 3 "~" H 1950 2100 50  0001 C CNN
	1    1950 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 1850 1950 1950
Connection ~ 1950 1850
Wire Wire Line
	1950 2250 1950 2350
Wire Wire Line
	1950 2350 1600 2350
Connection ~ 1600 2350
Wire Wire Line
	4550 1650 4550 1850
Connection ~ 4550 1850
Wire Wire Line
	2550 2400 3200 2400
Wire Wire Line
	2800 2550 2800 2800
Wire Wire Line
	2800 2800 2550 2800
Wire Wire Line
	2800 2550 3200 2550
Wire Wire Line
	4550 1850 5300 1850
$Comp
L Device:C C3
U 1 1 5E811437
P 2950 3100
F 0 "C3" H 3065 3146 50  0000 L CNN
F 1 "C" H 3065 3055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2988 2950 50  0001 C CNN
F 3 "~" H 2950 3100 50  0001 C CNN
	1    2950 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 2850 2950 2850
Wire Wire Line
	2950 2850 2950 2950
Wire Wire Line
	2950 3250 2950 3450
Connection ~ 2950 3450
Wire Wire Line
	2950 3450 2550 3450
$Comp
L Device:R R2
U 1 1 5E819F39
P 2550 2600
F 0 "R2" H 2620 2646 50  0000 L CNN
F 1 "R" H 2620 2555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2480 2600 50  0001 C CNN
F 3 "~" H 2550 2600 50  0001 C CNN
	1    2550 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5E81D3BF
P 2550 3100
F 0 "R3" H 2620 3146 50  0000 L CNN
F 1 "R" H 2620 3055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2480 3100 50  0001 C CNN
F 3 "~" H 2550 3100 50  0001 C CNN
	1    2550 3100
	1    0    0    -1  
$EndComp
Connection ~ 2550 2400
Wire Wire Line
	2550 2400 2550 2450
Wire Wire Line
	2550 2750 2550 2800
Wire Wire Line
	2550 2800 2550 2950
Connection ~ 2550 2800
Wire Wire Line
	2550 3250 2550 3450
Connection ~ 2550 3450
$Comp
L Device:R R4
U 1 1 5E82DAA6
P 4700 3250
F 0 "R4" H 4770 3296 50  0000 L CNN
F 1 "R" H 4770 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4630 3250 50  0001 C CNN
F 3 "~" H 4700 3250 50  0001 C CNN
	1    4700 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 3000 4700 3000
Wire Wire Line
	4700 3000 4700 3100
Wire Wire Line
	4700 3400 4700 3450
Wire Wire Line
	4700 3450 3750 3450
Connection ~ 3750 3450
Connection ~ 4700 3450
Wire Wire Line
	5300 2550 5300 1850
Connection ~ 5300 1850
Wire Wire Line
	5300 1850 6550 1850
$Comp
L Device:R R8
U 1 1 5E839631
P 5450 2900
F 0 "R8" H 5520 2946 50  0000 L CNN
F 1 "R" H 5520 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5380 2900 50  0001 C CNN
F 3 "~" H 5450 2900 50  0001 C CNN
	1    5450 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5E839D9E
P 5450 3250
F 0 "D2" V 5500 3100 50  0000 R CNN
F 1 " LED GREEN" V 5400 3150 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 5450 3250 50  0001 C CNN
F 3 "~" H 5450 3250 50  0001 C CNN
	1    5450 3250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5450 2700 5450 2750
Wire Wire Line
	5450 3050 5450 3100
Wire Wire Line
	5450 3400 5450 3450
Wire Wire Line
	4300 2700 5450 2700
$Comp
L Device:R R7
U 1 1 5E85CFAD
P 5100 3250
F 0 "R7" H 5170 3296 50  0000 L CNN
F 1 "R" H 5170 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5030 3250 50  0001 C CNN
F 3 "~" H 5100 3250 50  0001 C CNN
	1    5100 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2850 5100 2850
Wire Wire Line
	5100 2850 5100 3100
Wire Wire Line
	5100 3400 5100 3450
Connection ~ 5100 3450
Wire Wire Line
	5100 3450 5450 3450
Wire Wire Line
	2550 2250 2550 2400
Wire Wire Line
	2550 1850 3050 1850
Connection ~ 2550 1850
Wire Wire Line
	1950 1850 2550 1850
$Comp
L Device:R R1
U 1 1 5E816C1E
P 2550 2100
F 0 "R1" H 2620 2146 50  0000 L CNN
F 1 "R" H 2620 2055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2480 2100 50  0001 C CNN
F 3 "~" H 2550 2100 50  0001 C CNN
	1    2550 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 1850 2550 1950
NoConn ~ 3200 2700
$Comp
L Switch:SW_DIP_x01 SW1
U 1 1 5E8C35A9
P 2100 2900
F 0 "SW1" V 2150 3250 50  0000 R CNN
F 1 "SPST" V 2050 3250 50  0000 R CNN
F 2 "Button_Switch_SMD:SW_DIP_SPSTx01_Slide_Omron_A6S-110x_W8.9mm_P2.54mm" H 2100 2900 50  0001 C CNN
F 3 "https://eu.mouser.com/datasheet/2/307/en-a6s-1224092.pdf" H 2100 2900 50  0001 C CNN
F 4 "Omron" H 2100 2900 50  0001 C CNN "Manufacturer"
F 5 "A6S-1102-PH" H 2100 2900 50  0001 C CNN "MPN"
F 6 "Mouser" H 2100 2900 50  0001 C CNN "Distributor"
F 7 "653-A6S-1102-PH" H 2100 2900 50  0001 C CNN "Distributor PN"
	1    2100 2900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1300 3450 2100 3450
Wire Wire Line
	2100 2400 2100 2600
Wire Wire Line
	2100 2400 2550 2400
Wire Wire Line
	2100 3200 2100 3450
Connection ~ 2100 3450
Wire Wire Line
	2100 3450 2550 3450
Text Notes 1650 2700 0    50   ~ 0
ON/OFF
Text Notes 12550 3450 0    100  ~ 0
Fan Control
$Comp
L Device:R R6
U 1 1 5E902DB0
P 4950 2400
F 0 "R6" V 4743 2400 50  0000 C CNN
F 1 "R" V 4834 2400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4880 2400 50  0001 C CNN
F 3 "~" H 4950 2400 50  0001 C CNN
	1    4950 2400
	0    1    1    0   
$EndComp
$Comp
L Device:LED D1
U 1 1 5E902DB6
P 4600 2400
F 0 "D1" H 4593 2616 50  0000 C CNN
F 1 "LED RED" H 4593 2525 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 4600 2400 50  0001 C CNN
F 3 "~" H 4600 2400 50  0001 C CNN
	1    4600 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 2400 5100 2400
Wire Wire Line
	4800 2400 4750 2400
Wire Wire Line
	4300 2550 5300 2550
$Comp
L power:+BATT #PWR0109
U 1 1 5E91575B
P 5150 2200
F 0 "#PWR0109" H 5150 2050 50  0001 C CNN
F 1 "+BATT" H 5165 2373 50  0000 C CNN
F 2 "" H 5150 2200 50  0001 C CNN
F 3 "" H 5150 2200 50  0001 C CNN
	1    5150 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 2200 5150 2400
Wire Wire Line
	4300 2400 4450 2400
$Comp
L Device:C C4
U 1 1 5E9289BE
P 3700 9000
F 0 "C4" H 3815 9046 50  0000 L CNN
F 1 "C" H 3815 8955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3738 8850 50  0001 C CNN
F 3 "~" H 3700 9000 50  0001 C CNN
	1    3700 9000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5E9299CF
P 3700 9200
F 0 "#PWR0110" H 3700 8950 50  0001 C CNN
F 1 "GND" H 3705 9027 50  0000 C CNN
F 2 "" H 3700 9200 50  0001 C CNN
F 3 "" H 3700 9200 50  0001 C CNN
	1    3700 9200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 9200 3700 9150
Wire Wire Line
	3700 8850 3700 8750
Wire Wire Line
	3700 8750 3150 8750
Wire Wire Line
	3150 8750 3150 9300
$Comp
L power:GND #PWR0111
U 1 1 5E93213A
P 3150 10350
F 0 "#PWR0111" H 3150 10100 50  0001 C CNN
F 1 "GND" H 3155 10177 50  0000 C CNN
F 2 "" H 3150 10350 50  0001 C CNN
F 3 "" H 3150 10350 50  0001 C CNN
	1    3150 10350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 10200 3150 10350
Text Notes 2900 1250 0    118  ~ 0
Power Input eFuse
$Comp
L power:+5V #PWR0112
U 1 1 5E9512DF
P 6100 2000
F 0 "#PWR0112" H 6100 1850 50  0001 C CNN
F 1 "+5V" H 5950 2050 50  0000 C CNN
F 2 "" H 6100 2000 50  0001 C CNN
F 3 "" H 6100 2000 50  0001 C CNN
	1    6100 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 2050 6350 2050
Wire Wire Line
	6100 2050 6100 2000
$Comp
L power:GND #PWR0113
U 1 1 5E955A51
P 6450 1950
F 0 "#PWR0113" H 6450 1700 50  0001 C CNN
F 1 "GND" H 6455 1777 50  0000 C CNN
F 2 "" H 6450 1950 50  0001 C CNN
F 3 "" H 6450 1950 50  0001 C CNN
	1    6450 1950
	0    1    1    0   
$EndComp
Wire Wire Line
	6450 1950 6550 1950
$Comp
L power:GND #PWR0114
U 1 1 5E95A05E
P 8000 1750
F 0 "#PWR0114" H 8000 1500 50  0001 C CNN
F 1 "GND" H 8005 1577 50  0000 C CNN
F 2 "" H 8000 1750 50  0001 C CNN
F 3 "" H 8000 1750 50  0001 C CNN
	1    8000 1750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7850 1750 8000 1750
NoConn ~ 7850 1950
$Comp
L Device:R R10
U 1 1 5E969939
P 8400 1500
F 0 "R10" H 8470 1546 50  0000 L CNN
F 1 "R" H 8470 1455 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8330 1500 50  0001 C CNN
F 3 "~" H 8400 1500 50  0001 C CNN
	1    8400 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 5E96993F
P 8400 1850
F 0 "D3" V 8439 1732 50  0000 R CNN
F 1 " LED GREEN" V 8348 1732 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 8400 1850 50  0001 C CNN
F 3 "~" H 8400 1850 50  0001 C CNN
	1    8400 1850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8400 1650 8400 1700
Wire Wire Line
	7850 2050 8400 2050
Wire Wire Line
	8400 2050 8400 2000
$Comp
L power:+5V #PWR0115
U 1 1 5E970837
P 8400 1200
F 0 "#PWR0115" H 8400 1050 50  0001 C CNN
F 1 "+5V" H 8250 1250 50  0000 C CNN
F 2 "" H 8400 1200 50  0001 C CNN
F 3 "" H 8400 1200 50  0001 C CNN
	1    8400 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 1200 8400 1350
$Comp
L Device:R R5
U 1 1 5E9793F7
P 6300 1750
F 0 "R5" V 6093 1750 50  0000 C CNN
F 1 "0" V 6184 1750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6230 1750 50  0001 C CNN
F 3 "~" H 6300 1750 50  0001 C CNN
	1    6300 1750
	0    1    1    0   
$EndComp
Wire Wire Line
	6550 1750 6450 1750
Text Notes 6150 1450 0    50   ~ 0
Negative Logic\nOn/Off
$Comp
L Transistor_FET:BSS138 Q1
U 1 1 5E984287
P 5650 2450
F 0 "Q1" H 5854 2496 50  0000 L CNN
F 1 "BSS138" H 5854 2405 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5850 2375 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 5650 2450 50  0001 L CNN
	1    5650 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 2150 6350 2150
Wire Wire Line
	6350 2150 6350 2050
Connection ~ 6350 2050
Wire Wire Line
	6350 2050 6100 2050
Wire Wire Line
	5450 2450 5450 2700
Connection ~ 5450 2700
Wire Wire Line
	5750 2250 5750 1750
Wire Wire Line
	5750 1750 6150 1750
Wire Wire Line
	5750 2650 5750 3450
Wire Wire Line
	5750 3450 5450 3450
Connection ~ 5450 3450
$Comp
L Device:R R9
U 1 1 5E99C522
P 6450 2500
F 0 "R9" H 6520 2546 50  0000 L CNN
F 1 "1.3k" H 6520 2455 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6380 2500 50  0001 C CNN
F 3 "~" H 6450 2500 50  0001 C CNN
	1    6450 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 2250 6450 2250
Wire Wire Line
	6450 2250 6450 2350
$Comp
L power:GND #PWR0116
U 1 1 5E9A1367
P 6450 2750
F 0 "#PWR0116" H 6450 2500 50  0001 C CNN
F 1 "GND" H 6455 2577 50  0000 C CNN
F 2 "" H 6450 2750 50  0001 C CNN
F 3 "" H 6450 2750 50  0001 C CNN
	1    6450 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 2650 6450 2750
Wire Wire Line
	4700 3450 5100 3450
$Comp
L Device:R R11
U 1 1 5E9DE14F
P 1500 9450
F 0 "R11" H 1570 9496 50  0000 L CNN
F 1 "R" H 1570 9405 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1430 9450 50  0001 C CNN
F 3 "~" H 1500 9450 50  0001 C CNN
	1    1500 9450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 5E9DE5F8
P 1500 9950
F 0 "R12" H 1570 9996 50  0000 L CNN
F 1 "R" H 1570 9905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1430 9950 50  0001 C CNN
F 3 "~" H 1500 9950 50  0001 C CNN
	1    1500 9950
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0117
U 1 1 5E9DEB4D
P 1500 9200
F 0 "#PWR0117" H 1500 9050 50  0001 C CNN
F 1 "VBUS" H 1515 9373 50  0000 C CNN
F 2 "" H 1500 9200 50  0001 C CNN
F 3 "" H 1500 9200 50  0001 C CNN
	1    1500 9200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5E9DF0EC
P 1500 10200
F 0 "#PWR0118" H 1500 9950 50  0001 C CNN
F 1 "GND" H 1505 10027 50  0000 C CNN
F 2 "" H 1500 10200 50  0001 C CNN
F 3 "" H 1500 10200 50  0001 C CNN
	1    1500 10200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 9200 1500 9300
Wire Wire Line
	1500 9600 1500 9700
Wire Wire Line
	1500 10100 1500 10200
Wire Wire Line
	2750 9700 1500 9700
Connection ~ 1500 9700
Wire Wire Line
	1500 9700 1500 9800
Text Label 4450 2850 0    50   ~ 0
IMON
Text Label 2400 9800 2    50   ~ 0
IMON
Wire Wire Line
	2400 9800 2750 9800
Text Label 2400 9700 2    50   ~ 0
VMON
$Comp
L Connector:Conn_01x03_Male J3
U 1 1 5EA2E7C9
P 13000 2200
F 0 "J3" H 12972 2132 50  0000 R CNN
F 1 "Conn_01x03_Male" H 12972 2223 50  0000 R CNN
F 2 "Connector_JST:JST_PH_B3B-PH-K_1x03_P2.00mm_Vertical" H 13000 2200 50  0001 C CNN
F 3 "~" H 13000 2200 50  0001 C CNN
	1    13000 2200
	-1   0    0    1   
$EndComp
Text Notes 12600 1600 0    100  ~ 0
Servo Connector
$Comp
L power:+5V #PWR0119
U 1 1 5EA31AB6
P 12600 1850
F 0 "#PWR0119" H 12600 1700 50  0001 C CNN
F 1 "+5V" H 12450 1900 50  0000 C CNN
F 2 "" H 12600 1850 50  0001 C CNN
F 3 "" H 12600 1850 50  0001 C CNN
	1    12600 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5EA32081
P 12600 2450
F 0 "#PWR0120" H 12600 2200 50  0001 C CNN
F 1 "GND" H 12605 2277 50  0000 C CNN
F 2 "" H 12600 2450 50  0001 C CNN
F 3 "" H 12600 2450 50  0001 C CNN
	1    12600 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	12800 2300 12600 2300
Wire Wire Line
	12600 2300 12600 2450
Wire Wire Line
	12800 2100 12600 2100
Wire Wire Line
	12600 2100 12600 1850
Wire Wire Line
	12800 2200 12100 2200
Text Label 12100 2200 0    50   ~ 0
SERVO_PWM
$Comp
L Connector:Conn_01x02_Male J4
U 1 1 5EA40C30
P 13000 4050
F 0 "J4" H 12972 3932 50  0000 R CNN
F 1 "Conn_01x02_Male" H 12972 4023 50  0000 R CNN
F 2 "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical" H 13000 4050 50  0001 C CNN
F 3 "~" H 13000 4050 50  0001 C CNN
	1    13000 4050
	-1   0    0    1   
$EndComp
$Comp
L Transistor_FET:DMN10H220L Q2
U 1 1 5EA42ADB
P 12500 4400
F 0 "Q2" H 12704 4446 50  0000 L CNN
F 1 "DMN10H220L" H 12704 4355 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 12700 4325 50  0001 L CIN
F 3 "http://www.diodes.com/assets/Datasheets/DMN10H220L.pdf" H 12500 4400 50  0001 L CNN
	1    12500 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	12800 4050 12600 4050
Wire Wire Line
	12600 4050 12600 4200
$Comp
L power:+5V #PWR0121
U 1 1 5EA4A816
P 12600 3750
F 0 "#PWR0121" H 12600 3600 50  0001 C CNN
F 1 "+5V" H 12450 3800 50  0000 C CNN
F 2 "" H 12600 3750 50  0001 C CNN
F 3 "" H 12600 3750 50  0001 C CNN
	1    12600 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	12800 3950 12600 3950
Wire Wire Line
	12600 3950 12600 3750
$Comp
L power:GND #PWR0122
U 1 1 5EA4FF43
P 12600 4750
F 0 "#PWR0122" H 12600 4500 50  0001 C CNN
F 1 "GND" H 12605 4577 50  0000 C CNN
F 2 "" H 12600 4750 50  0001 C CNN
F 3 "" H 12600 4750 50  0001 C CNN
	1    12600 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	12600 4600 12600 4750
Text Label 11700 4400 0    50   ~ 0
FAN_CTRL
Wire Wire Line
	11700 4400 12300 4400
Text Notes 11800 9950 0    50   ~ 0
Todo:\n* Add additional eFuse for motor\n* Add motor controller\n* Add in start module connector & outline for placement\n* Change to horizontal connectors\n* Add connector for hall-sensor\n* Decide on connector for motor\n* Add connector & PMOS for TOF sensor
$Comp
L Connector:Raspberry_Pi_2_3 J5
U 1 1 5E7C70F3
P 10050 6900
F 0 "J5" H 9300 8300 50  0000 C CNN
F 1 "Raspberry_Pi" H 9500 8200 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x20_P2.54mm_Vertical" H 10050 6900 50  0001 C CNN
F 3 "https://www.raspberrypi.org/documentation/hardware/raspberrypi/schematics/rpi_SCH_3bplus_1p0_reduced.pdf" H 10050 6900 50  0001 C CNN
	1    10050 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 5600 9850 5500
Wire Wire Line
	9950 5600 9950 5500
Wire Wire Line
	9950 5500 9850 5500
Connection ~ 9850 5500
Wire Wire Line
	9850 5500 9850 5400
Wire Wire Line
	9650 8200 9650 8250
Wire Wire Line
	9650 8250 9750 8250
Wire Wire Line
	9750 8250 9750 8200
Connection ~ 9650 8250
Wire Wire Line
	9650 8250 9650 8350
Wire Wire Line
	9850 8200 9850 8250
Wire Wire Line
	9850 8250 9750 8250
Connection ~ 9750 8250
Wire Wire Line
	9950 8200 9950 8250
Wire Wire Line
	9950 8250 9850 8250
Connection ~ 9850 8250
Wire Wire Line
	10350 8200 10350 8250
Wire Wire Line
	10350 8250 10250 8250
Connection ~ 9950 8250
Wire Wire Line
	10050 8200 10050 8250
Connection ~ 10050 8250
Wire Wire Line
	10050 8250 9950 8250
Wire Wire Line
	10150 8200 10150 8250
Connection ~ 10150 8250
Wire Wire Line
	10150 8250 10050 8250
Wire Wire Line
	10250 8200 10250 8250
Connection ~ 10250 8250
Wire Wire Line
	10250 8250 10150 8250
Wire Wire Line
	10150 5400 10150 5500
Wire Wire Line
	10250 5600 10250 5500
Wire Wire Line
	10250 5500 10150 5500
Connection ~ 10150 5500
Wire Wire Line
	10150 5500 10150 5600
$Comp
L power:+3V3 #PWR0102
U 1 1 5E83D1BD
P 3150 8600
F 0 "#PWR0102" H 3150 8450 50  0001 C CNN
F 1 "+3V3" H 3165 8773 50  0000 C CNN
F 2 "" H 3150 8600 50  0001 C CNN
F 3 "" H 3150 8600 50  0001 C CNN
	1    3150 8600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 8600 3150 8750
Connection ~ 3150 8750
$Comp
L DCDC:DRV8874PWPR IC1
U 1 1 5E7B2154
P 4750 5300
F 0 "IC1" H 5450 5565 50  0000 C CNN
F 1 "DRV8874PWPR" H 5450 5474 50  0000 C CNN
F 2 "DCDC:SOP65P640X120-17N" H 6000 5400 50  0001 L CNN
F 3 "https://www.ti.com/lit/gpn/DRV8874" H 6000 5300 50  0001 L CNN
F 4 "H-Bridge Motor Driver With Integrated Current Sense and Regulation" H 6000 5200 50  0001 L CNN "Description"
F 5 "1.2" H 6000 5100 50  0001 L CNN "Height"
F 6 "595-DRV8874PWPR" H 6000 5000 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.com/Search/Refine.aspx?Keyword=595-DRV8874PWPR" H 6000 4900 50  0001 L CNN "Mouser Price/Stock"
F 8 "Texas Instruments" H 6000 4800 50  0001 L CNN "Manufacturer_Name"
F 9 "DRV8874PWPR" H 6000 4700 50  0001 L CNN "Manufacturer_Part_Number"
	1    4750 5300
	1    0    0    -1  
$EndComp
$EndSCHEMATC
