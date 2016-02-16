EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:sr086
LIBS:rectifier-bridge
LIBS:encoder
LIBS:crystal_s
LIBS:varibulb-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CP1 C2
U 1 1 56B67334
P 5700 1300
F 0 "C2" H 5833 1346 50  0000 L CNN
F 1 "470u" H 5750 1150 50  0000 L CNN
F 2 "" H 5700 1300 60  0000 C CNN
F 3 "" H 5700 1300 60  0000 C CNN
	1    5700 1300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 56B6744D
P 5700 1550
F 0 "#PWR01" H 5700 1300 60  0001 C CNN
F 1 "GND" H 5708 1369 60  0001 C CNN
F 2 "" H 5700 1550 60  0000 C CNN
F 3 "" H 5700 1550 60  0000 C CNN
	1    5700 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 56B681AC
P 2750 2150
F 0 "#PWR02" H 2750 1900 60  0001 C CNN
F 1 "GND" H 2758 1969 60  0001 C CNN
F 2 "" H 2750 2150 60  0000 C CNN
F 3 "" H 2750 2150 60  0000 C CNN
	1    2750 2150
	1    0    0    -1  
$EndComp
$Comp
L APE8865N-33-HF-3 U1
U 1 1 56B692B2
P 7050 1000
F 0 "U1" H 7250 1200 40  0000 C CNN
F 1 "LDO 5V" H 7250 750 40  0000 C CNN
F 2 "SSOP5" H 7050 1216 35  0000 C CIN
F 3 "" H 7050 1000 60  0000 C CNN
	1    7050 1000
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 56B69533
P 6450 1300
F 0 "C3" H 6400 1200 40  0000 C CNN
F 1 "100n" H 6350 1400 40  0000 C CNN
F 2 "" H 6488 1150 30  0000 C CNN
F 3 "" H 6450 1300 60  0000 C CNN
	1    6450 1300
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR03
U 1 1 56B695A0
P 6450 1550
F 0 "#PWR03" H 6450 1300 60  0001 C CNN
F 1 "GND" H 6458 1369 60  0001 C CNN
F 2 "" H 6450 1550 60  0000 C CNN
F 3 "" H 6450 1550 60  0000 C CNN
	1    6450 1550
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 56B69775
P 7650 1300
F 0 "C4" H 7600 1200 40  0000 C CNN
F 1 "100n" H 7550 1400 40  0000 C CNN
F 2 "" H 7688 1150 30  0000 C CNN
F 3 "" H 7650 1300 60  0000 C CNN
	1    7650 1300
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR04
U 1 1 56B6977B
P 7650 1550
F 0 "#PWR04" H 7650 1300 60  0001 C CNN
F 1 "GND" H 7658 1369 60  0001 C CNN
F 2 "" H 7650 1550 60  0000 C CNN
F 3 "" H 7650 1550 60  0000 C CNN
	1    7650 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 56B697E5
P 7050 1550
F 0 "#PWR05" H 7050 1300 60  0001 C CNN
F 1 "GND" H 7058 1369 60  0001 C CNN
F 2 "" H 7050 1550 60  0000 C CNN
F 3 "" H 7050 1550 60  0000 C CNN
	1    7050 1550
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR06
U 1 1 56B69A4D
P 7650 850
F 0 "#PWR06" H 7650 700 60  0001 C CNN
F 1 "VCC" H 7670 1031 60  0000 C CNN
F 2 "" H 7650 850 60  0000 C CNN
F 3 "" H 7650 850 60  0000 C CNN
	1    7650 850 
	1    0    0    -1  
$EndComp
$Comp
L ATTINY2313A-P IC1
U 1 1 56B69E24
P 6200 4200
F 0 "IC1" H 7250 5200 40  0000 C CNN
F 1 "ATTINY2313A-P" H 7100 3300 40  0000 C CNN
F 2 "DIP20" H 6450 3300 35  0000 C CIN
F 3 "" H 6200 4200 60  0000 C CNN
	1    6200 4200
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR07
U 1 1 56B6A04C
P 6200 3050
F 0 "#PWR07" H 6200 2900 60  0001 C CNN
F 1 "VCC" H 6220 3231 60  0000 C CNN
F 2 "" H 6200 3050 60  0000 C CNN
F 3 "" H 6200 3050 60  0000 C CNN
	1    6200 3050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 56B6A16B
P 6200 5250
F 0 "#PWR08" H 6200 5000 60  0001 C CNN
F 1 "GND" H 6208 5069 60  0001 C CNN
F 2 "" H 6200 5250 60  0000 C CNN
F 3 "" H 6200 5250 60  0000 C CNN
	1    6200 5250
	1    0    0    -1  
$EndComp
Text Label 3250 1550 0    60   ~ 0
SENSE
$Comp
L TRIAC U2
U 1 1 56B6B84E
P 2000 3150
F 0 "U2" H 2328 3225 70  0000 L CNN
F 1 "TRIAC" H 2327 3165 60  0001 L CNN
F 2 "" H 2000 3150 60  0000 C CNN
F 3 "" H 2000 3150 60  0000 C CNN
	1    2000 3150
	-1   0    0    -1  
$EndComp
$Comp
L TRIAC U3
U 1 1 56B6B9EC
P 2000 4150
F 0 "U3" H 2328 4225 70  0000 L CNN
F 1 "TRIAC" H 2327 4165 60  0001 L CNN
F 2 "" H 2000 4150 60  0000 C CNN
F 3 "" H 2000 4150 60  0000 C CNN
	1    2000 4150
	-1   0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 56B6BAF0
P 2850 3350
F 0 "R6" V 2750 3350 40  0000 C CNN
F 1 "100" V 2850 3350 40  0000 C CNN
F 2 "" V 2780 3350 30  0000 C CNN
F 3 "" H 2850 3350 30  0000 C CNN
	1    2850 3350
	0    1    1    0   
$EndComp
Text Label 3150 3350 0    60   ~ 0
SW1
Text Label 3150 4350 0    60   ~ 0
SW2
Text Label 7600 3700 0    60   ~ 0
SW1
Text Label 7600 3800 0    60   ~ 0
SW2
$Comp
L ENCODER SW1
U 1 1 56B6CD24
P 10450 4800
F 0 "SW1" H 10450 5100 60  0000 C CNN
F 1 "ENCODER" H 10450 4500 60  0000 C CNN
F 2 "" H 10450 4800 60  0000 C CNN
F 3 "" H 10450 4800 60  0000 C CNN
	1    10450 4800
	-1   0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 56B6CF55
P 9400 5200
F 0 "C9" H 9515 5239 40  0000 L CNN
F 1 "10n" H 9515 5161 40  0000 L CNN
F 2 "" H 9438 5050 30  0000 C CNN
F 3 "" H 9400 5200 60  0000 C CNN
	1    9400 5200
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 56B6CFF5
P 9000 5200
F 0 "C8" H 9115 5239 40  0000 L CNN
F 1 "10n" H 9115 5161 40  0000 L CNN
F 2 "" H 9038 5050 30  0000 C CNN
F 3 "" H 9000 5200 60  0000 C CNN
	1    9000 5200
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 56B6D1BA
P 9000 4350
F 0 "R10" H 9071 4389 40  0000 L CNN
F 1 "10k" H 9071 4311 40  0000 L CNN
F 2 "" V 8930 4350 30  0000 C CNN
F 3 "" H 9000 4350 30  0000 C CNN
	1    9000 4350
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 56B6D2EB
P 9400 4350
F 0 "R11" H 9471 4389 40  0000 L CNN
F 1 "10k" H 9471 4311 40  0000 L CNN
F 2 "" V 9330 4350 30  0000 C CNN
F 3 "" H 9400 4350 30  0000 C CNN
	1    9400 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 950  5700 1100
Wire Wire Line
	5700 1550 5700 1500
Connection ~ 5700 950 
Wire Wire Line
	6450 1550 6450 1500
Wire Wire Line
	6450 950  6450 1100
Connection ~ 6450 950 
Wire Wire Line
	7650 1550 7650 1500
Wire Wire Line
	7650 850  7650 950 
Wire Wire Line
	7650 950  7650 1100
Wire Wire Line
	7650 950  7500 950 
Wire Wire Line
	7050 1550 7050 1300
Connection ~ 7650 950 
Wire Wire Line
	6200 3050 6200 3100
Wire Wire Line
	6200 5200 6200 5250
Wire Wire Line
	7600 3800 7550 3800
Wire Wire Line
	7550 3700 7600 3700
Wire Wire Line
	9400 4600 9400 4800
Wire Wire Line
	9400 4800 9400 5000
Wire Wire Line
	9000 4600 9000 4700
Wire Wire Line
	9000 4700 9000 5000
$Comp
L GND #PWR09
U 1 1 56B6D801
P 9000 5450
F 0 "#PWR09" H 9000 5200 60  0001 C CNN
F 1 "GND" H 9008 5269 60  0001 C CNN
F 2 "" H 9000 5450 60  0000 C CNN
F 3 "" H 9000 5450 60  0000 C CNN
	1    9000 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 5450 9000 5400
$Comp
L GND #PWR010
U 1 1 56B6D8E2
P 9400 5450
F 0 "#PWR010" H 9400 5200 60  0001 C CNN
F 1 "GND" H 9408 5269 60  0001 C CNN
F 2 "" H 9400 5450 60  0000 C CNN
F 3 "" H 9400 5450 60  0000 C CNN
	1    9400 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 5450 9400 5400
$Comp
L VCC #PWR011
U 1 1 56B6DA69
P 9000 4050
F 0 "#PWR011" H 9000 3900 60  0001 C CNN
F 1 "VCC" H 9020 4231 60  0000 C CNN
F 2 "" H 9000 4050 60  0000 C CNN
F 3 "" H 9000 4050 60  0000 C CNN
	1    9000 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 4050 9000 4100
$Comp
L VCC #PWR012
U 1 1 56B6DC33
P 9400 4050
F 0 "#PWR012" H 9400 3900 60  0001 C CNN
F 1 "VCC" H 9420 4231 60  0000 C CNN
F 2 "" H 9400 4050 60  0000 C CNN
F 3 "" H 9400 4050 60  0000 C CNN
	1    9400 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 4050 9400 4100
$Comp
L GND #PWR013
U 1 1 56B6DE98
P 11100 4900
F 0 "#PWR013" H 11100 4650 60  0001 C CNN
F 1 "GND" H 11108 4719 60  0001 C CNN
F 2 "" H 11100 4900 60  0000 C CNN
F 3 "" H 11100 4900 60  0000 C CNN
	1    11100 4900
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 56B6E820
P 4350 3400
F 0 "R7" V 4250 3400 40  0000 C CNN
F 1 "10k" V 4350 3400 40  0000 C CNN
F 2 "" V 4280 3400 30  0000 C CNN
F 3 "" H 4350 3400 30  0000 C CNN
	1    4350 3400
	0    1    1    0   
$EndComp
Wire Wire Line
	4600 3400 4750 3400
Wire Wire Line
	4750 3400 4850 3400
$Comp
L VCC #PWR014
U 1 1 56B6EC93
P 4050 3400
F 0 "#PWR014" H 4050 3250 60  0001 C CNN
F 1 "VCC" V 4071 3527 60  0000 L CNN
F 2 "" H 4050 3400 60  0000 C CNN
F 3 "" H 4050 3400 60  0000 C CNN
	1    4050 3400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4050 3400 4100 3400
$Comp
L CONN_02X03 P7
U 1 1 56B6EE7F
P 8050 5900
F 0 "P7" H 8050 6216 50  0000 C CNN
F 1 "ISP" H 8050 6124 50  0000 C CNN
F 2 "" H 8050 4700 60  0000 C CNN
F 3 "" H 8050 4700 60  0000 C CNN
	1    8050 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 5800 7750 5800
Wire Wire Line
	7750 5800 7750 4000
Wire Wire Line
	7750 4000 7550 4000
Wire Wire Line
	7800 5900 7650 5900
Wire Wire Line
	7650 5900 7650 4100
Wire Wire Line
	7650 4100 7550 4100
Wire Wire Line
	8300 5900 8400 5900
Wire Wire Line
	8400 5900 8400 5500
Wire Wire Line
	8400 5500 7850 5500
Wire Wire Line
	7850 5500 7850 3900
Wire Wire Line
	7850 3900 7550 3900
Wire Wire Line
	7800 6000 4750 6000
Wire Wire Line
	4750 6000 4750 3400
Connection ~ 4750 3400
$Comp
L VCC #PWR015
U 1 1 56B6F861
P 8500 5800
F 0 "#PWR015" H 8500 5650 60  0001 C CNN
F 1 "VCC" V 8520 5928 60  0000 L CNN
F 2 "" H 8500 5800 60  0000 C CNN
F 3 "" H 8500 5800 60  0000 C CNN
	1    8500 5800
	0    1    1    0   
$EndComp
$Comp
L GND #PWR016
U 1 1 56B6F93E
P 8500 6000
F 0 "#PWR016" H 8500 5750 60  0001 C CNN
F 1 "GND" H 8508 5819 60  0001 C CNN
F 2 "" H 8500 6000 60  0000 C CNN
F 3 "" H 8500 6000 60  0000 C CNN
	1    8500 6000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8500 6000 8300 6000
Wire Wire Line
	8500 5800 8300 5800
Text Label 7600 3400 0    60   ~ 0
SENSE
$Comp
L CONN_01X04 P5
U 1 1 56B6FF72
P 8650 4350
F 0 "P5" H 8600 4600 50  0000 L CNN
F 1 "COMM" H 8550 4100 50  0000 L CNN
F 2 "" H 8650 4350 60  0000 C CNN
F 3 "" H 8650 4350 60  0000 C CNN
	1    8650 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 4500 7950 4500
Wire Wire Line
	7950 4500 7950 4300
Connection ~ 7950 4300
$Comp
L VCC #PWR017
U 1 1 56B701EB
P 8400 4200
F 0 "#PWR017" H 8400 4050 60  0001 C CNN
F 1 "VCC" V 8421 4327 60  0000 L CNN
F 2 "" H 8400 4200 60  0000 C CNN
F 3 "" H 8400 4200 60  0000 C CNN
	1    8400 4200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8400 4200 8450 4200
$Comp
L GND #PWR018
U 1 1 56B70439
P 8400 4500
F 0 "#PWR018" H 8400 4250 60  0001 C CNN
F 1 "GND" H 8408 4319 60  0001 C CNN
F 2 "" H 8400 4500 60  0000 C CNN
F 3 "" H 8400 4500 60  0000 C CNN
	1    8400 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	8400 4500 8450 4500
$Comp
L CRYSTAL_S X1
U 1 1 56B717CE
P 4050 4250
F 0 "X1" H 4250 4300 60  0000 C CNN
F 1 "10MHz" H 4250 4050 60  0000 C CNN
F 2 "" H 4050 4250 60  0000 C CNN
F 3 "" H 4050 4250 60  0000 C CNN
	1    4050 4250
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 56B71AB6
P 3650 4600
F 0 "C6" H 3765 4639 40  0000 L CNN
F 1 "22p" H 3765 4561 40  0000 L CNN
F 2 "" H 3688 4450 30  0000 C CNN
F 3 "" H 3650 4600 60  0000 C CNN
	1    3650 4600
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 56B71C3A
P 4450 4600
F 0 "C7" H 4565 4639 40  0000 L CNN
F 1 "22p" H 4565 4561 40  0000 L CNN
F 2 "" H 4488 4450 30  0000 C CNN
F 3 "" H 4450 4600 60  0000 C CNN
	1    4450 4600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 56B71E37
P 3650 4850
F 0 "#PWR019" H 3650 4600 60  0001 C CNN
F 1 "GND" H 3658 4669 60  0001 C CNN
F 2 "" H 3650 4850 60  0000 C CNN
F 3 "" H 3650 4850 60  0000 C CNN
	1    3650 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 4850 3650 4800
$Comp
L GND #PWR020
U 1 1 56B71FD7
P 4450 4850
F 0 "#PWR020" H 4450 4600 60  0001 C CNN
F 1 "GND" H 4458 4669 60  0001 C CNN
F 2 "" H 4450 4850 60  0000 C CNN
F 3 "" H 4450 4850 60  0000 C CNN
	1    4450 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 4850 4450 4800
$Comp
L GND #PWR021
U 1 1 56B720F0
P 4050 4850
F 0 "#PWR021" H 4050 4600 60  0001 C CNN
F 1 "GND" H 4058 4669 60  0001 C CNN
F 2 "" H 4050 4850 60  0000 C CNN
F 3 "" H 4050 4850 60  0000 C CNN
	1    4050 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 4850 4050 4500
Wire Wire Line
	4450 3900 4450 4250
Wire Wire Line
	4450 4250 4450 4400
Wire Wire Line
	4450 3900 4850 3900
Wire Wire Line
	4850 3700 3650 3700
Wire Wire Line
	3650 3700 3650 4250
Wire Wire Line
	3650 4250 3650 4400
Wire Wire Line
	3750 4250 3650 4250
Connection ~ 3650 4250
Wire Wire Line
	4350 4250 4450 4250
Connection ~ 4450 4250
$Comp
L R R12
U 1 1 56B7372F
P 9800 4350
F 0 "R12" H 9871 4389 40  0000 L CNN
F 1 "10k" H 9871 4311 40  0000 L CNN
F 2 "" V 9730 4350 30  0000 C CNN
F 3 "" H 9800 4350 30  0000 C CNN
	1    9800 4350
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR022
U 1 1 56B737C3
P 9800 4050
F 0 "#PWR022" H 9800 3900 60  0001 C CNN
F 1 "VCC" H 9820 4231 60  0000 C CNN
F 2 "" H 9800 4050 60  0000 C CNN
F 3 "" H 9800 4050 60  0000 C CNN
	1    9800 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 4050 9800 4100
$Comp
L C C10
U 1 1 56B738FF
P 9800 5200
F 0 "C10" H 9915 5239 40  0000 L CNN
F 1 "10n" H 9915 5161 40  0000 L CNN
F 2 "" H 9838 5050 30  0000 C CNN
F 3 "" H 9800 5200 60  0000 C CNN
	1    9800 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 4600 9800 4900
Wire Wire Line
	9800 4900 9800 5000
Wire Wire Line
	7550 4900 9800 4900
Wire Wire Line
	9800 4900 9850 4900
Connection ~ 9800 4900
Wire Wire Line
	7550 4800 9400 4800
Wire Wire Line
	9400 4800 9850 4800
Connection ~ 9400 4800
Wire Wire Line
	7550 4700 9000 4700
Wire Wire Line
	9000 4700 9850 4700
Connection ~ 9000 4700
$Comp
L GND #PWR023
U 1 1 56B741FE
P 9800 5450
F 0 "#PWR023" H 9800 5200 60  0001 C CNN
F 1 "GND" H 9808 5269 60  0001 C CNN
F 2 "" H 9800 5450 60  0000 C CNN
F 3 "" H 9800 5450 60  0000 C CNN
	1    9800 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 5450 9800 5400
$Comp
L FUSE F1
U 1 1 56B74EEA
P 2300 950
F 0 "F1" H 2400 1000 40  0000 C CNN
F 1 "FUSE" H 2250 850 40  0000 C CNN
F 2 "" H 2300 950 60  0000 C CNN
F 3 "" H 2300 950 60  0000 C CNN
	1    2300 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 950  2000 950 
Wire Wire Line
	2000 950  2050 950 
Wire Wire Line
	2550 950  2750 950 
Wire Wire Line
	2750 950  3150 950 
$Comp
L ZENER D4
U 1 1 56B6AA5E
P 3500 1850
F 0 "D4" V 3539 1771 50  0000 R CNN
F 1 "4v" V 3454 1771 40  0000 R CNN
F 2 "" H 3500 1850 60  0000 C CNN
F 3 "" H 3500 1850 60  0000 C CNN
	1    3500 1850
	0    -1   -1   0   
$EndComp
$Comp
L R R3
U 1 1 56BF6F40
P 2750 1250
F 0 "R3" H 2821 1289 40  0000 L CNN
F 1 "2.7M" H 2821 1211 40  0000 L CNN
F 2 "" V 2680 1250 30  0000 C CNN
F 3 "" H 2750 1250 30  0000 C CNN
	1    2750 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 1000 2750 950 
Connection ~ 2750 950 
$Comp
L R R4
U 1 1 56BF7120
P 2750 1850
F 0 "R4" H 2821 1889 40  0000 L CNN
F 1 "560k" H 2821 1811 40  0000 L CNN
F 2 "" V 2680 1850 30  0000 C CNN
F 3 "" H 2750 1850 30  0000 C CNN
	1    2750 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 1500 2750 1550
Wire Wire Line
	2750 1550 2750 1600
Wire Wire Line
	2750 2150 2750 2100
$Comp
L GND #PWR024
U 1 1 56BF762F
P 3500 2150
F 0 "#PWR024" H 3500 1900 60  0001 C CNN
F 1 "GND" H 3508 1969 60  0001 C CNN
F 2 "" H 3500 2150 60  0000 C CNN
F 3 "" H 3500 2150 60  0000 C CNN
	1    3500 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 2150 3500 2050
Wire Wire Line
	3500 1550 3500 1650
Wire Wire Line
	2750 1550 3100 1550
Wire Wire Line
	3100 1550 3500 1550
Connection ~ 2750 1550
$Comp
L C C5
U 1 1 56C03CA0
P 3100 1850
F 0 "C5" H 3215 1889 40  0000 L CNN
F 1 "100p" H 3215 1811 40  0000 L CNN
F 2 "" H 3138 1700 30  0000 C CNN
F 3 "" H 3100 1850 60  0000 C CNN
	1    3100 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 1650 3100 1550
Connection ~ 3100 1550
$Comp
L GND #PWR025
U 1 1 56C03FDF
P 3100 2150
F 0 "#PWR025" H 3100 1900 60  0001 C CNN
F 1 "GND" H 3108 1969 60  0001 C CNN
F 2 "" H 3100 2150 60  0000 C CNN
F 3 "" H 3100 2150 60  0000 C CNN
	1    3100 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 2150 3100 2050
$Comp
L R R1
U 1 1 56C044CF
P 3400 950
F 0 "R1" V 3400 950 40  0000 C CNN
F 1 "47k" V 3500 950 40  0000 C CNN
F 2 "" V 3330 950 30  0000 C CNN
F 3 "" H 3400 950 30  0000 C CNN
	1    3400 950 
	0    1    1    0   
$EndComp
$Comp
L C C1
U 1 1 56C0476F
P 4150 950
F 0 "C1" V 4100 1050 40  0000 L CNN
F 1 "220n" V 4250 1050 40  0000 L CNN
F 2 "" H 4188 800 30  0000 C CNN
F 3 "" H 4150 950 60  0000 C CNN
	1    4150 950 
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 56C2411C
P 4150 1200
F 0 "R2" V 4150 1200 40  0000 C CNN
F 1 "1M" V 4250 1200 40  0000 C CNN
F 2 "" V 4080 1200 30  0000 C CNN
F 3 "" H 4150 1200 30  0000 C CNN
	1    4150 1200
	0    1    1    0   
$EndComp
$Comp
L DIODE D3
U 1 1 56C2476A
P 4750 1250
F 0 "D3" V 4750 1172 40  0000 R CNN
F 1 "DIODE" V 4711 1172 40  0001 R CNN
F 2 "" H 4750 1250 60  0000 C CNN
F 3 "" H 4750 1250 60  0000 C CNN
	1    4750 1250
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR026
U 1 1 56C249B9
P 4750 1550
F 0 "#PWR026" H 4750 1300 60  0001 C CNN
F 1 "GND" H 4758 1369 60  0001 C CNN
F 2 "" H 4750 1550 60  0000 C CNN
F 3 "" H 4750 1550 60  0000 C CNN
	1    4750 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 1550 4750 1450
$Comp
L DIODE D1
U 1 1 56C24BD0
P 5050 950
F 0 "D1" H 5150 1000 40  0000 C CNN
F 1 "DIODE" V 5011 872 40  0001 R CNN
F 2 "" H 5050 950 60  0000 C CNN
F 3 "" H 5050 950 60  0000 C CNN
	1    5050 950 
	1    0    0    -1  
$EndComp
$Comp
L ZENER D2
U 1 1 56C24E58
P 5350 1250
F 0 "D2" V 5389 1171 50  0000 R CNN
F 1 "7v" V 5304 1171 40  0000 R CNN
F 2 "" H 5350 1250 60  0000 C CNN
F 3 "" H 5350 1250 60  0000 C CNN
	1    5350 1250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5350 1050 5350 950 
Wire Wire Line
	5250 950  5350 950 
Wire Wire Line
	5350 950  5700 950 
Wire Wire Line
	5700 950  6450 950 
Wire Wire Line
	6450 950  6600 950 
$Comp
L GND #PWR027
U 1 1 56C24FEB
P 5350 1550
F 0 "#PWR027" H 5350 1300 60  0001 C CNN
F 1 "GND" H 5358 1369 60  0001 C CNN
F 2 "" H 5350 1550 60  0000 C CNN
F 3 "" H 5350 1550 60  0000 C CNN
	1    5350 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 1550 5350 1450
$Comp
L CONNECTOR P1
U 1 1 56C266E3
P 1900 950
F 0 "P1" H 2100 800 70  0000 C CNN
F 1 "MAINS LIVE" H 2600 1100 70  0000 C CNN
F 2 "" H 1900 950 60  0000 C CNN
F 3 "" H 1900 950 60  0000 C CNN
	1    1900 950 
	-1   0    0    1   
$EndComp
$Comp
L CONNECTOR P6
U 1 1 56C26F29
P 1900 5600
F 0 "P6" H 2100 5450 70  0000 C CNN
F 1 "MAINS GROUND" H 2700 5600 70  0000 C CNN
F 2 "" H 1900 5600 60  0000 C CNN
F 3 "" H 1900 5600 60  0000 C CNN
	1    1900 5600
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR028
U 1 1 56C271A4
P 2000 5700
F 0 "#PWR028" H 2000 5450 60  0001 C CNN
F 1 "GND" H 2008 5519 60  0001 C CNN
F 2 "" H 2000 5700 60  0000 C CNN
F 3 "" H 2000 5700 60  0000 C CNN
	1    2000 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 3350 2500 3350
$Comp
L CONNECTOR P3
U 1 1 56C27947
P 1900 2650
F 0 "P3" H 2100 2500 70  0000 C CNN
F 1 "LAMP 1" H 2500 2650 70  0000 C CNN
F 2 "" H 1900 2650 60  0000 C CNN
F 3 "" H 1900 2650 60  0000 C CNN
	1    1900 2650
	-1   0    0    1   
$EndComp
Wire Wire Line
	2000 2750 2000 2650
Wire Wire Line
	2000 2650 1900 2650
$Comp
L GND #PWR029
U 1 1 56C27C85
P 2000 3450
F 0 "#PWR029" H 2000 3200 60  0001 C CNN
F 1 "GND" H 2008 3269 60  0001 C CNN
F 2 "" H 2000 3450 60  0000 C CNN
F 3 "" H 2000 3450 60  0000 C CNN
	1    2000 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 3450 2000 3400
$Comp
L CONNECTOR P4
U 1 1 56C280B6
P 1900 3650
F 0 "P4" H 2100 3500 70  0000 C CNN
F 1 "LAMP 2" H 2500 3650 70  0000 C CNN
F 2 "" H 1900 3650 60  0000 C CNN
F 3 "" H 1900 3650 60  0000 C CNN
	1    1900 3650
	-1   0    0    1   
$EndComp
Wire Wire Line
	1900 3650 2000 3650
Wire Wire Line
	2000 3650 2000 3750
$Comp
L GND #PWR030
U 1 1 56C2820D
P 2000 4450
F 0 "#PWR030" H 2000 4200 60  0001 C CNN
F 1 "GND" H 2008 4269 60  0001 C CNN
F 2 "" H 2000 4450 60  0000 C CNN
F 3 "" H 2000 4450 60  0000 C CNN
	1    2000 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 4450 2000 4400
$Comp
L R R9
U 1 1 56C28356
P 2850 4350
F 0 "R9" V 2750 4350 40  0000 C CNN
F 1 "100" V 2850 4350 40  0000 C CNN
F 2 "" V 2780 4350 30  0000 C CNN
F 3 "" H 2850 4350 30  0000 C CNN
	1    2850 4350
	0    1    1    0   
$EndComp
Wire Wire Line
	2600 4350 2500 4350
Wire Wire Line
	2000 5700 2000 5600
Wire Wire Line
	2000 5600 1900 5600
$Comp
L CONNECTOR P2
U 1 1 56C28A1B
P 1900 1250
F 0 "P2" H 2100 1100 70  0000 C CNN
F 1 "MAINS LIVE" H 2600 1250 70  0001 C CNN
F 2 "" H 1900 1250 60  0000 C CNN
F 3 "" H 1900 1250 60  0000 C CNN
	1    1900 1250
	-1   0    0    1   
$EndComp
Wire Wire Line
	1900 1250 2000 1250
Wire Wire Line
	2000 1250 2000 950 
Connection ~ 2000 950 
Wire Wire Line
	3150 3350 3100 3350
Wire Wire Line
	3150 4350 3100 4350
Wire Notes Line
	650  6200 3500 6200
Wire Notes Line
	3500 6200 3500 2600
Wire Notes Line
	3500 2600 5550 2600
Wire Notes Line
	5550 2600 5550 700 
Wire Notes Line
	5550 700  650  700 
Wire Notes Line
	650  700  650  6200
Wire Wire Line
	7550 4300 7950 4300
Wire Wire Line
	7950 4300 8450 4300
Wire Wire Line
	8450 4400 7550 4400
Wire Wire Line
	11100 4900 11100 4800
Wire Wire Line
	11100 4800 11050 4800
NoConn ~ 7550 3600
$Comp
L R R5
U 1 1 56C2DF82
P 10250 3200
F 0 "R5" H 10321 3239 40  0000 L CNN
F 1 "10k" H 10321 3161 40  0000 L CNN
F 2 "" V 10180 3200 30  0000 C CNN
F 3 "" H 10250 3200 30  0000 C CNN
	1    10250 3200
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 56C2E04B
P 10250 3800
F 0 "R8" H 10321 3839 40  0000 L CNN
F 1 "10k" H 10321 3761 40  0000 L CNN
F 2 "" V 10180 3800 30  0000 C CNN
F 3 "" H 10250 3800 30  0000 C CNN
	1    10250 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 3500 10250 3500
Wire Wire Line
	10250 3450 10250 3500
Wire Wire Line
	10250 3500 10250 3550
Connection ~ 10250 3500
$Comp
L GND #PWR031
U 1 1 56C2E54E
P 10250 4100
F 0 "#PWR031" H 10250 3850 60  0001 C CNN
F 1 "GND" H 10258 3919 60  0001 C CNN
F 2 "" H 10250 4100 60  0000 C CNN
F 3 "" H 10250 4100 60  0000 C CNN
	1    10250 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 4100 10250 4050
$Comp
L VCC #PWR032
U 1 1 56C2E7B2
P 10250 2900
F 0 "#PWR032" H 10250 2750 60  0001 C CNN
F 1 "VCC" H 10270 3081 60  0000 C CNN
F 2 "" H 10250 2900 60  0000 C CNN
F 3 "" H 10250 2900 60  0000 C CNN
	1    10250 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 2900 10250 2950
Wire Wire Line
	7550 3400 7600 3400
NoConn ~ 7550 4600
Connection ~ 5350 950 
Wire Wire Line
	3900 1200 3850 1200
Wire Wire Line
	3850 1200 3850 950 
Wire Wire Line
	3650 950  3850 950 
Wire Wire Line
	3850 950  3950 950 
Connection ~ 3850 950 
Wire Wire Line
	4400 1200 4450 1200
Wire Wire Line
	4450 1200 4450 950 
Wire Wire Line
	4350 950  4450 950 
Wire Wire Line
	4450 950  4750 950 
Wire Wire Line
	4750 950  4850 950 
Wire Wire Line
	4750 950  4750 1050
Connection ~ 4450 950 
Connection ~ 4750 950 
$EndSCHEMATC
