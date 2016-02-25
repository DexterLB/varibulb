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
L CP1 C5
U 1 1 56B67334
P 5700 1300
F 0 "C5" H 5833 1346 50  0000 L CNN
F 1 "470μ" H 5750 1150 50  0000 L CNN
F 2 "Capacitors_ThroughHole:Capacitor10x16RM5" H 5700 1300 60  0001 C CNN
F 3 "" H 5700 1300 60  0000 C CNN
	1    5700 1300
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 56B69533
P 6300 1300
F 0 "C6" H 6250 1200 40  0000 C CNN
F 1 "100n" H 6200 1400 40  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 6338 1150 30  0001 C CNN
F 3 "" H 6300 1300 60  0000 C CNN
	1    6300 1300
	-1   0    0    1   
$EndComp
$Comp
L C C7
U 1 1 56B69775
P 7500 1300
F 0 "C7" H 7450 1200 40  0000 C CNN
F 1 "100n" H 7400 1400 40  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 7538 1150 30  0001 C CNN
F 3 "" H 7500 1300 60  0000 C CNN
	1    7500 1300
	-1   0    0    1   
$EndComp
$Comp
L VCC #PWR01
U 1 1 56B6A04C
P 6200 2900
F 0 "#PWR01" H 6200 2750 60  0001 C CNN
F 1 "VCC" H 7000 2800 60  0000 C CNN
F 2 "" H 6200 2900 60  0000 C CNN
F 3 "" H 6200 2900 60  0000 C CNN
	1    6200 2900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 56B6A16B
P 6200 5100
F 0 "#PWR02" H 6200 4850 60  0001 C CNN
F 1 "GND" H 6208 4919 60  0001 C CNN
F 2 "" H 6200 5100 60  0000 C CNN
F 3 "" H 6200 5100 60  0000 C CNN
	1    6200 5100
	1    0    0    -1  
$EndComp
$Comp
L TRIAC U1
U 1 1 56B6B84E
P 2000 3150
F 0 "U1" H 2328 3225 70  0000 L CNN
F 1 "TRIAC" H 2327 3165 60  0001 L CNN
F 2 "Transistors_TO-220:TO-220_Neutral123_Vertical" H 2000 3150 60  0001 C CNN
F 3 "" H 2000 3150 60  0000 C CNN
	1    2000 3150
	-1   0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 56B6BAF0
P 2850 3350
F 0 "R3" V 2750 3350 40  0000 C CNN
F 1 "470Ω" V 2850 3350 40  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 2780 3350 30  0001 C CNN
F 3 "" H 2850 3350 30  0000 C CNN
	1    2850 3350
	0    1    1    0   
$EndComp
$Comp
L ENCODER SW1
U 1 1 56B6CD24
P 10450 4650
F 0 "SW1" H 10450 5100 60  0000 C CNN
F 1 "ENCODER" H 10450 4350 60  0000 C CNN
F 2 "encoder:ENCODER" H 10450 4650 60  0001 C CNN
F 3 "" H 10450 4650 60  0000 C CNN
	1    10450 4650
	-1   0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 56B6CF55
P 9400 5050
F 0 "C9" H 9515 5089 40  0000 L CNN
F 1 "10n" H 9515 5011 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 9438 4900 30  0001 C CNN
F 3 "" H 9400 5050 60  0000 C CNN
	1    9400 5050
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 56B6CFF5
P 9000 5050
F 0 "C8" H 9115 5089 40  0000 L CNN
F 1 "10n" H 9115 5011 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 9038 4900 30  0001 C CNN
F 3 "" H 9000 5050 60  0000 C CNN
	1    9000 5050
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 56B6D1BA
P 9000 4200
F 0 "R9" H 9071 4239 40  0000 L CNN
F 1 "10k" H 9071 4161 40  0000 L CNN
F 2 "Resistors_SMD:R_0805" V 8930 4200 30  0001 C CNN
F 3 "" H 9000 4200 30  0000 C CNN
	1    9000 4200
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 56B6D2EB
P 9400 4200
F 0 "R10" H 9471 4239 40  0000 L CNN
F 1 "10k" H 9471 4161 40  0000 L CNN
F 2 "Resistors_SMD:R_0805" V 9330 4200 30  0001 C CNN
F 3 "" H 9400 4200 30  0000 C CNN
	1    9400 4200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 56B6D801
P 9000 5300
F 0 "#PWR03" H 9000 5050 60  0001 C CNN
F 1 "GND" H 9008 5119 60  0001 C CNN
F 2 "" H 9000 5300 60  0000 C CNN
F 3 "" H 9000 5300 60  0000 C CNN
	1    9000 5300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 56B6D8E2
P 9400 5300
F 0 "#PWR04" H 9400 5050 60  0001 C CNN
F 1 "GND" H 9408 5119 60  0001 C CNN
F 2 "" H 9400 5300 60  0000 C CNN
F 3 "" H 9400 5300 60  0000 C CNN
	1    9400 5300
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR05
U 1 1 56B6DA69
P 9000 3900
F 0 "#PWR05" H 9000 3750 60  0001 C CNN
F 1 "VCC" H 9020 4081 60  0000 C CNN
F 2 "" H 9000 3900 60  0000 C CNN
F 3 "" H 9000 3900 60  0000 C CNN
	1    9000 3900
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR06
U 1 1 56B6DC33
P 9400 3900
F 0 "#PWR06" H 9400 3750 60  0001 C CNN
F 1 "VCC" H 9420 4081 60  0000 C CNN
F 2 "" H 9400 3900 60  0000 C CNN
F 3 "" H 9400 3900 60  0000 C CNN
	1    9400 3900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 56B6DE98
P 11100 4850
F 0 "#PWR07" H 11100 4600 60  0001 C CNN
F 1 "GND" H 11108 4669 60  0001 C CNN
F 2 "" H 11100 4850 60  0000 C CNN
F 3 "" H 11100 4850 60  0000 C CNN
	1    11100 4850
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 56B6E820
P 4350 3250
F 0 "R7" V 4250 3250 40  0000 C CNN
F 1 "10k" V 4350 3250 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 4280 3250 30  0001 C CNN
F 3 "" H 4350 3250 30  0000 C CNN
	1    4350 3250
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR08
U 1 1 56B6EC93
P 4050 3250
F 0 "#PWR08" H 4050 3100 60  0001 C CNN
F 1 "VCC" V 4071 3377 60  0000 L CNN
F 2 "" H 4050 3250 60  0000 C CNN
F 3 "" H 4050 3250 60  0000 C CNN
	1    4050 3250
	0    -1   -1   0   
$EndComp
$Comp
L CONN_02X03 P6
U 1 1 56B6EE7F
P 8050 6300
F 0 "P6" H 8050 6500 50  0000 C CNN
F 1 "ISP" H 8300 6500 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03" H 8050 5100 60  0001 C CNN
F 3 "" H 8050 5100 60  0000 C CNN
	1    8050 6300
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR09
U 1 1 56B6F861
P 8500 6200
F 0 "#PWR09" H 8500 6050 60  0001 C CNN
F 1 "VCC" V 8520 6328 60  0000 L CNN
F 2 "" H 8500 6200 60  0000 C CNN
F 3 "" H 8500 6200 60  0000 C CNN
	1    8500 6200
	0    1    1    0   
$EndComp
$Comp
L GND #PWR010
U 1 1 56B6F93E
P 8500 6400
F 0 "#PWR010" H 8500 6150 60  0001 C CNN
F 1 "GND" H 8508 6219 60  0001 C CNN
F 2 "" H 8500 6400 60  0000 C CNN
F 3 "" H 8500 6400 60  0000 C CNN
	1    8500 6400
	0    -1   -1   0   
$EndComp
Text Label 7600 3250 0    60   ~ 0
SENSE
$Comp
L CONN_01X04 P7
U 1 1 56B6FF72
P 8650 4200
F 0 "P7" H 8600 4450 50  0000 L CNN
F 1 "COMM" H 8550 3950 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 8650 4200 60  0001 C CNN
F 3 "" H 8650 4200 60  0000 C CNN
	1    8650 4200
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR011
U 1 1 56B701EB
P 8400 4050
F 0 "#PWR011" H 8400 3900 60  0001 C CNN
F 1 "VCC" V 8421 4177 60  0000 L CNN
F 2 "" H 8400 4050 60  0000 C CNN
F 3 "" H 8400 4050 60  0000 C CNN
	1    8400 4050
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR012
U 1 1 56B70439
P 8400 4350
F 0 "#PWR012" H 8400 4100 60  0001 C CNN
F 1 "GND" H 8408 4169 60  0001 C CNN
F 2 "" H 8400 4350 60  0000 C CNN
F 3 "" H 8400 4350 60  0000 C CNN
	1    8400 4350
	0    1    1    0   
$EndComp
$Comp
L CRYSTAL_S X1
U 1 1 56B717CE
P 4050 4100
F 0 "X1" H 4250 4150 60  0000 C CNN
F 1 "10MHz" H 4250 3900 60  0000 C CNN
F 2 "HC-49V:HC-49V" H 4050 4100 60  0001 C CNN
F 3 "" H 4050 4100 60  0000 C CNN
	1    4050 4100
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 56B71AB6
P 3650 4450
F 0 "C2" H 3765 4489 40  0000 L CNN
F 1 "22p" H 3765 4411 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3688 4300 30  0001 C CNN
F 3 "" H 3650 4450 60  0000 C CNN
	1    3650 4450
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 56B71C3A
P 4450 4450
F 0 "C4" H 4565 4489 40  0000 L CNN
F 1 "22p" H 4565 4411 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4488 4300 30  0001 C CNN
F 3 "" H 4450 4450 60  0000 C CNN
	1    4450 4450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 56B71E37
P 3650 4700
F 0 "#PWR013" H 3650 4450 60  0001 C CNN
F 1 "GND" H 3658 4519 60  0001 C CNN
F 2 "" H 3650 4700 60  0000 C CNN
F 3 "" H 3650 4700 60  0000 C CNN
	1    3650 4700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 56B71FD7
P 4450 4700
F 0 "#PWR014" H 4450 4450 60  0001 C CNN
F 1 "GND" H 4458 4519 60  0001 C CNN
F 2 "" H 4450 4700 60  0000 C CNN
F 3 "" H 4450 4700 60  0000 C CNN
	1    4450 4700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 56B720F0
P 4050 4700
F 0 "#PWR015" H 4050 4450 60  0001 C CNN
F 1 "GND" H 4058 4519 60  0001 C CNN
F 2 "" H 4050 4700 60  0000 C CNN
F 3 "" H 4050 4700 60  0000 C CNN
	1    4050 4700
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 56B7372F
P 9800 4200
F 0 "R11" H 9871 4239 40  0000 L CNN
F 1 "10k" H 9871 4161 40  0000 L CNN
F 2 "Resistors_SMD:R_0805" V 9730 4200 30  0001 C CNN
F 3 "" H 9800 4200 30  0000 C CNN
	1    9800 4200
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR016
U 1 1 56B737C3
P 9800 3900
F 0 "#PWR016" H 9800 3750 60  0001 C CNN
F 1 "VCC" H 9820 4081 60  0000 C CNN
F 2 "" H 9800 3900 60  0000 C CNN
F 3 "" H 9800 3900 60  0000 C CNN
	1    9800 3900
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 56B738FF
P 9800 5050
F 0 "C10" H 9915 5089 40  0000 L CNN
F 1 "10n" H 9915 5011 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 9838 4900 30  0001 C CNN
F 3 "" H 9800 5050 60  0000 C CNN
	1    9800 5050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 56B741FE
P 9800 5300
F 0 "#PWR017" H 9800 5050 60  0001 C CNN
F 1 "GND" H 9808 5119 60  0001 C CNN
F 2 "" H 9800 5300 60  0000 C CNN
F 3 "" H 9800 5300 60  0000 C CNN
	1    9800 5300
	1    0    0    -1  
$EndComp
$Comp
L FUSE F1
U 1 1 56B74EEA
P 2250 950
F 0 "F1" H 2350 1000 40  0000 C CNN
F 1 "FUSE" H 2200 850 40  0000 C CNN
F 2 "Fuse_Holders_and_Fuses:Fuseholder5x20_horiz_open_lateral_Type-II" H 2250 950 60  0001 C CNN
F 3 "" H 2250 950 60  0000 C CNN
	1    2250 950 
	1    0    0    -1  
$EndComp
$Comp
L ZENER D3
U 1 1 56B6AA5E
P 3500 2100
F 0 "D3" V 3539 2021 50  0000 R CNN
F 1 "3.3v" V 3454 2021 40  0000 R CNN
F 2 "Diodes_ThroughHole:Diode_DO-35_SOD27_Horizontal_RM10" H 3500 2100 60  0001 C CNN
F 3 "" H 3500 2100 60  0000 C CNN
	1    3500 2100
	0    -1   -1   0   
$EndComp
$Comp
L R R1
U 1 1 56BF6F40
P 2750 1500
F 0 "R1" H 2821 1539 40  0000 L CNN
F 1 "2.7M" H 2821 1461 40  0000 L CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 2680 1500 30  0001 C CNN
F 3 "" H 2750 1500 30  0000 C CNN
	1    2750 1500
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 56BF7120
P 2750 2100
F 0 "R2" H 2821 2139 40  0000 L CNN
F 1 "560k" H 2821 2061 40  0000 L CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 2680 2100 30  0001 C CNN
F 3 "" H 2750 2100 30  0000 C CNN
	1    2750 2100
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 56C03CA0
P 3100 2100
F 0 "C1" H 3215 2139 40  0000 L CNN
F 1 "100p" H 3215 2061 40  0000 L CNN
F 2 "Capacitors_ThroughHole:Capacitor6MMDiscRM5" H 3138 1950 30  0001 C CNN
F 3 "" H 3100 2100 60  0000 C CNN
	1    3100 2100
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 56C044CF
P 3400 950
F 0 "R5" V 3400 950 40  0000 C CNN
F 1 "510Ω" V 3500 950 40  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM15mm" V 3330 950 30  0001 C CNN
F 3 "" H 3400 950 30  0000 C CNN
	1    3400 950 
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 56C0476F
P 4150 950
F 0 "C3" V 4100 1050 40  0000 L CNN
F 1 "220n" V 4250 1050 40  0000 L CNN
F 2 "Capacitors_ThroughHole:Capacitor13x6x12mmRM10" H 4188 800 30  0001 C CNN
F 3 "" H 4150 950 60  0000 C CNN
	1    4150 950 
	0    1    1    0   
$EndComp
$Comp
L R R6
U 1 1 56C2411C
P 4150 1200
F 0 "R6" V 4150 1200 40  0000 C CNN
F 1 "1M" V 4250 1200 40  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 4080 1200 30  0001 C CNN
F 3 "" H 4150 1200 30  0000 C CNN
	1    4150 1200
	0    1    1    0   
$EndComp
$Comp
L DIODE D4
U 1 1 56C2476A
P 4750 1250
F 0 "D4" V 4750 1172 40  0000 R CNN
F 1 "DIODE" V 4711 1172 40  0001 R CNN
F 2 "Diodes_ThroughHole:Diode_DO-35_SOD27_Horizontal_RM10" H 4750 1250 60  0001 C CNN
F 3 "" H 4750 1250 60  0000 C CNN
	1    4750 1250
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D5
U 1 1 56C24BD0
P 5050 950
F 0 "D5" H 5150 1000 40  0000 C CNN
F 1 "DIODE" V 5011 872 40  0001 R CNN
F 2 "Diodes_ThroughHole:Diode_DO-35_SOD27_Horizontal_RM10" H 5050 950 60  0001 C CNN
F 3 "" H 5050 950 60  0000 C CNN
	1    5050 950 
	1    0    0    -1  
$EndComp
$Comp
L ZENER D6
U 1 1 56C24E58
P 5350 1250
F 0 "D6" V 5389 1171 50  0000 R CNN
F 1 "8.2v" V 5250 1200 40  0000 R CNN
F 2 "Diodes_ThroughHole:Diode_DO-35_SOD27_Horizontal_RM10" H 5350 1250 60  0001 C CNN
F 3 "" H 5350 1250 60  0000 C CNN
	1    5350 1250
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR018
U 1 1 56C24FEB
P 9050 1050
F 0 "#PWR018" H 9050 800 60  0001 C CNN
F 1 "GND" H 9058 869 60  0001 C CNN
F 2 "" H 9050 1050 60  0000 C CNN
F 3 "" H 9050 1050 60  0000 C CNN
	1    9050 1050
	0    -1   -1   0   
$EndComp
$Comp
L CONNECTOR P1
U 1 1 56C266E3
P 1900 950
F 0 "P1" H 2100 800 70  0000 C CNN
F 1 "MAINS LIVE" H 2600 1100 70  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 1900 950 60  0001 C CNN
F 3 "" H 1900 950 60  0000 C CNN
	1    1900 950 
	-1   0    0    1   
$EndComp
$Comp
L CONNECTOR P5
U 1 1 56C26F29
P 1900 5600
F 0 "P5" H 2100 5450 70  0000 C CNN
F 1 "MAINS GROUND" H 2700 5600 70  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 1900 5600 60  0001 C CNN
F 3 "" H 1900 5600 60  0000 C CNN
	1    1900 5600
	-1   0    0    1   
$EndComp
NoConn ~ 7550 3450
$Comp
L R R12
U 1 1 56C2DF82
P 10250 3050
F 0 "R12" H 10321 3089 40  0000 L CNN
F 1 "10k" H 10321 3011 40  0000 L CNN
F 2 "Resistors_SMD:R_0805" V 10180 3050 30  0001 C CNN
F 3 "" H 10250 3050 30  0000 C CNN
	1    10250 3050
	1    0    0    -1  
$EndComp
$Comp
L R R13
U 1 1 56C2E04B
P 10250 3650
F 0 "R13" H 10321 3689 40  0000 L CNN
F 1 "10k" H 10321 3611 40  0000 L CNN
F 2 "Resistors_SMD:R_0805" V 10180 3650 30  0001 C CNN
F 3 "" H 10250 3650 30  0000 C CNN
	1    10250 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 56C2E54E
P 10250 3950
F 0 "#PWR019" H 10250 3700 60  0001 C CNN
F 1 "GND" H 10258 3769 60  0001 C CNN
F 2 "" H 10250 3950 60  0000 C CNN
F 3 "" H 10250 3950 60  0000 C CNN
	1    10250 3950
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR020
U 1 1 56C2E7B2
P 10250 2750
F 0 "#PWR020" H 10250 2600 60  0001 C CNN
F 1 "VCC" H 10270 2931 60  0000 C CNN
F 2 "" H 10250 2750 60  0000 C CNN
F 3 "" H 10250 2750 60  0000 C CNN
	1    10250 2750
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 56C3207A
P 8050 5050
F 0 "R8" H 8121 5089 40  0000 L CNN
F 1 "1k" H 8121 5011 40  0000 L CNN
F 2 "Resistors_SMD:R_0805" V 7980 5050 30  0001 C CNN
F 3 "" H 8050 5050 30  0000 C CNN
	1    8050 5050
	1    0    0    -1  
$EndComp
$Comp
L LED D7
U 1 1 56C322FB
P 8050 5550
F 0 "D7" V 8004 5628 50  0000 L CNN
F 1 "LED" V 8096 5628 50  0000 L CNN
F 2 "Diodes_ThroughHole:Diode_LED_3mm_round_vertical" H 8050 5550 60  0001 C CNN
F 3 "" H 8050 5550 60  0000 C CNN
	1    8050 5550
	0    1    1    0   
$EndComp
$Comp
L GND #PWR021
U 1 1 56C32631
P 8050 5800
F 0 "#PWR021" H 8050 5550 60  0001 C CNN
F 1 "GND" H 8058 5619 60  0001 C CNN
F 2 "" H 8050 5800 60  0000 C CNN
F 3 "" H 8050 5800 60  0000 C CNN
	1    8050 5800
	1    0    0    -1  
$EndComp
$Comp
L ZENER D1
U 1 1 56C396A8
P 3150 3600
F 0 "D1" V 3189 3521 50  0000 R CNN
F 1 "5v" V 3104 3521 40  0000 R CNN
F 2 "Diodes_ThroughHole:Diode_DO-35_SOD27_Horizontal_RM10" H 3150 3600 60  0001 C CNN
F 3 "" H 3150 3600 60  0000 C CNN
	1    3150 3600
	0    -1   -1   0   
$EndComp
$Comp
L ATTINY2313A-S IC1
U 1 1 56CB6933
P 6200 4050
F 0 "IC1" H 6200 5378 40  0000 C CNN
F 1 "ATTINY2313A-S" H 7000 3150 40  0000 C CNN
F 2 "Housings_SOIC:SOIC-20_7.5x12.8mm_Pitch1.27mm" H 6550 3150 35  0000 C CIN
F 3 "" H 6200 4050 60  0000 C CNN
	1    6200 4050
	1    0    0    -1  
$EndComp
NoConn ~ 7550 3650
$Comp
L CONN_01X04 P2
U 1 1 56CB75C3
P 8100 1100
F 0 "P2" H 8050 1350 50  0000 L CNN
F 1 "BOARD" H 8000 850 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 8100 1100 60  0001 C CNN
F 3 "" H 8100 1100 60  0000 C CNN
	1    8100 1100
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P3
U 1 1 56CB87D2
P 8500 1100
F 0 "P3" H 8450 1350 50  0000 L CNN
F 1 "BOARD" H 8400 850 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 8500 1100 60  0001 C CNN
F 3 "" H 8500 1100 60  0000 C CNN
	1    8500 1100
	-1   0    0    -1  
$EndComp
$Comp
L GNDREF #PWR022
U 1 1 56CB8CE9
P 7800 1050
F 0 "#PWR022" H 7800 800 60  0001 C CNN
F 1 "GNDREF" H 7808 869 60  0001 C CNN
F 2 "" H 7800 1050 60  0000 C CNN
F 3 "" H 7800 1050 60  0000 C CNN
	1    7800 1050
	0    1    1    0   
$EndComp
Text Label 8750 1150 0    60   ~ 0
SENSE
Text Label 8750 1250 0    60   ~ 0
PHASE
Text Label 7600 3550 0    60   ~ 0
PHASE
$Comp
L GNDREF #PWR023
U 1 1 56CB991B
P 5700 1550
F 0 "#PWR023" H 5700 1300 60  0001 C CNN
F 1 "GNDREF" H 5708 1369 60  0001 C CNN
F 2 "" H 5700 1550 60  0000 C CNN
F 3 "" H 5700 1550 60  0000 C CNN
	1    5700 1550
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR024
U 1 1 56CB9A16
P 5350 1550
F 0 "#PWR024" H 5350 1300 60  0001 C CNN
F 1 "GNDREF" H 5358 1369 60  0001 C CNN
F 2 "" H 5350 1550 60  0000 C CNN
F 3 "" H 5350 1550 60  0000 C CNN
	1    5350 1550
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR025
U 1 1 56CB9A99
P 4750 1550
F 0 "#PWR025" H 4750 1300 60  0001 C CNN
F 1 "GNDREF" H 4758 1369 60  0001 C CNN
F 2 "" H 4750 1550 60  0000 C CNN
F 3 "" H 4750 1550 60  0000 C CNN
	1    4750 1550
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR026
U 1 1 56CB9B1C
P 3500 2400
F 0 "#PWR026" H 3500 2150 60  0001 C CNN
F 1 "GNDREF" H 3508 2219 60  0001 C CNN
F 2 "" H 3500 2400 60  0000 C CNN
F 3 "" H 3500 2400 60  0000 C CNN
	1    3500 2400
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR027
U 1 1 56CB9B9F
P 3100 2400
F 0 "#PWR027" H 3100 2150 60  0001 C CNN
F 1 "GNDREF" H 3108 2219 60  0001 C CNN
F 2 "" H 3100 2400 60  0000 C CNN
F 3 "" H 3100 2400 60  0000 C CNN
	1    3100 2400
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR028
U 1 1 56CB9C72
P 2750 2400
F 0 "#PWR028" H 2750 2150 60  0001 C CNN
F 1 "GNDREF" H 2758 2219 60  0001 C CNN
F 2 "" H 2750 2400 60  0000 C CNN
F 3 "" H 2750 2400 60  0000 C CNN
	1    2750 2400
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR029
U 1 1 56CB9CF5
P 3150 3850
F 0 "#PWR029" H 3150 3600 60  0001 C CNN
F 1 "GNDREF" H 3158 3669 60  0001 C CNN
F 2 "" H 3150 3850 60  0000 C CNN
F 3 "" H 3150 3850 60  0000 C CNN
	1    3150 3850
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR030
U 1 1 56CB9DF0
P 2000 5700
F 0 "#PWR030" H 2000 5450 60  0001 C CNN
F 1 "GNDREF" H 2008 5519 60  0001 C CNN
F 2 "" H 2000 5700 60  0000 C CNN
F 3 "" H 2000 5700 60  0000 C CNN
	1    2000 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 950  5700 1100
Wire Wire Line
	5700 1550 5700 1500
Connection ~ 5700 950 
Wire Wire Line
	6300 1550 6300 1500
Wire Wire Line
	6300 950  6300 1100
Connection ~ 6300 950 
Wire Wire Line
	7500 1550 7500 1500
Wire Wire Line
	7500 950  7500 1100
Wire Wire Line
	7300 950  7900 950 
Wire Wire Line
	6900 1550 6900 1250
Connection ~ 7500 950 
Wire Wire Line
	6200 2900 6200 2950
Wire Wire Line
	6200 5050 6200 5100
Wire Wire Line
	7550 3550 7600 3550
Wire Wire Line
	9400 4450 9400 4850
Wire Wire Line
	9000 4450 9000 4850
Wire Wire Line
	9000 5300 9000 5250
Wire Wire Line
	9400 5300 9400 5250
Wire Wire Line
	9000 3900 9000 3950
Wire Wire Line
	9400 3900 9400 3950
Wire Wire Line
	4600 3250 4850 3250
Wire Wire Line
	4050 3250 4100 3250
Wire Wire Line
	7800 6200 7750 6200
Wire Wire Line
	7750 6200 7750 3850
Wire Wire Line
	7750 3850 7550 3850
Wire Wire Line
	7800 6300 7650 6300
Wire Wire Line
	7650 6300 7650 3950
Wire Wire Line
	7650 3950 7550 3950
Wire Wire Line
	8300 6300 8400 6300
Wire Wire Line
	8400 6300 8400 5950
Wire Wire Line
	8400 5950 7850 5950
Wire Wire Line
	7850 5950 7850 3750
Wire Wire Line
	7850 3750 7550 3750
Wire Wire Line
	7800 6400 4750 6400
Wire Wire Line
	4750 6400 4750 3250
Connection ~ 4750 3250
Wire Wire Line
	8500 6400 8300 6400
Wire Wire Line
	8500 6200 8300 6200
Wire Wire Line
	7550 4350 7950 4350
Wire Wire Line
	7950 4350 7950 4150
Connection ~ 7950 4150
Wire Wire Line
	8400 4050 8450 4050
Wire Wire Line
	8400 4350 8450 4350
Wire Wire Line
	3650 4700 3650 4650
Wire Wire Line
	4450 4700 4450 4650
Wire Wire Line
	4050 4700 4050 4350
Wire Wire Line
	4450 3750 4450 4250
Wire Wire Line
	4450 3750 4850 3750
Wire Wire Line
	4850 3550 3650 3550
Wire Wire Line
	3650 3550 3650 4250
Wire Wire Line
	3750 4100 3650 4100
Connection ~ 3650 4100
Wire Wire Line
	4350 4100 4450 4100
Connection ~ 4450 4100
Wire Wire Line
	9800 3900 9800 3950
Wire Wire Line
	9800 4450 9800 4850
Wire Wire Line
	7550 4750 9850 4750
Connection ~ 9800 4750
Connection ~ 9400 4650
Connection ~ 9000 4550
Wire Wire Line
	9800 5300 9800 5250
Wire Wire Line
	1900 950  2000 950 
Wire Wire Line
	2500 950  3150 950 
Wire Wire Line
	2750 1250 2750 950 
Connection ~ 2750 950 
Wire Wire Line
	2750 1750 2750 1850
Wire Wire Line
	2750 2400 2750 2350
Wire Wire Line
	3500 2400 3500 2300
Wire Wire Line
	3500 1800 3500 1900
Wire Wire Line
	2750 1800 7750 1800
Connection ~ 2750 1800
Wire Wire Line
	3100 1900 3100 1800
Connection ~ 3100 1800
Wire Wire Line
	3100 2400 3100 2300
Wire Wire Line
	4750 1550 4750 1450
Wire Wire Line
	5350 1050 5350 950 
Wire Wire Line
	5250 950  6500 950 
Wire Wire Line
	5350 1550 5350 1450
Wire Wire Line
	2600 3350 2500 3350
Wire Wire Line
	2000 1250 2000 2750
Wire Wire Line
	2000 5600 1900 5600
Wire Notes Line
	650  6200 3500 6200
Wire Notes Line
	3500 6200 3500 2600
Wire Notes Line
	650  700  650  6200
Wire Wire Line
	7550 4150 8450 4150
Wire Wire Line
	8450 4250 7550 4250
Wire Wire Line
	7550 3350 10250 3350
Wire Wire Line
	10250 3300 10250 3400
Connection ~ 10250 3350
Wire Wire Line
	10250 3950 10250 3900
Wire Wire Line
	10250 2750 10250 2800
Wire Wire Line
	7550 3250 7600 3250
Connection ~ 5350 950 
Wire Wire Line
	3900 1200 3850 1200
Wire Wire Line
	3850 1200 3850 950 
Wire Wire Line
	3650 950  3950 950 
Connection ~ 3850 950 
Wire Wire Line
	4400 1200 4450 1200
Wire Wire Line
	4450 1200 4450 950 
Wire Wire Line
	4350 950  4850 950 
Wire Wire Line
	4750 950  4750 1050
Connection ~ 4450 950 
Connection ~ 4750 950 
Wire Wire Line
	8050 4800 8050 4450
Wire Wire Line
	8050 4450 7550 4450
Wire Wire Line
	8050 5350 8050 5300
Wire Wire Line
	8050 5800 8050 5750
Wire Wire Line
	2550 1250 2550 950 
Connection ~ 2550 950 
Wire Wire Line
	3150 2550 3150 3400
Connection ~ 3150 3350
Wire Wire Line
	3150 3850 3150 3800
Wire Wire Line
	11100 4600 11100 4850
Wire Wire Line
	11100 4600 11050 4600
Wire Wire Line
	11050 4750 11100 4750
Connection ~ 11100 4750
Wire Wire Line
	2550 1250 2000 1250
Connection ~ 3500 1800
Wire Wire Line
	3150 2550 7850 2550
Wire Wire Line
	3150 3350 3100 3350
Wire Notes Line
	3500 2600 8250 2600
Wire Notes Line
	650  700  8250 700 
Wire Wire Line
	8700 1050 9050 1050
Wire Wire Line
	8700 1150 8750 1150
Connection ~ 2000 5600
Wire Wire Line
	2000 3400 2000 5700
Wire Wire Line
	8750 1250 8700 1250
Wire Wire Line
	7800 1050 7900 1050
Wire Wire Line
	7750 1800 7750 1150
Wire Wire Line
	7750 1150 7900 1150
Wire Wire Line
	7900 1250 7850 1250
Wire Wire Line
	7850 1250 7850 2550
$Comp
L VCC #PWR031
U 1 1 56CBBE9A
P 9600 950
F 0 "#PWR031" H 9600 800 60  0001 C CNN
F 1 "VCC" V 9450 1050 60  0000 C CNN
F 2 "" H 9600 950 60  0000 C CNN
F 3 "" H 9600 950 60  0000 C CNN
	1    9600 950 
	0    1    1    0   
$EndComp
Wire Wire Line
	8700 950  9600 950 
$Comp
L C C11
U 1 1 56CBC117
P 9500 1300
F 0 "C11" H 9450 1200 40  0000 C CNN
F 1 "100n" H 9400 1400 40  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 9538 1150 30  0001 C CNN
F 3 "" H 9500 1300 60  0000 C CNN
	1    9500 1300
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR032
U 1 1 56CBC328
P 9500 1550
F 0 "#PWR032" H 9500 1300 60  0001 C CNN
F 1 "GND" H 9508 1369 60  0001 C CNN
F 2 "" H 9500 1550 60  0000 C CNN
F 3 "" H 9500 1550 60  0000 C CNN
	1    9500 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 1550 9500 1500
Wire Wire Line
	9500 1100 9500 950 
Connection ~ 9500 950 
$Comp
L GNDREF #PWR033
U 1 1 56CBC641
P 6300 1550
F 0 "#PWR033" H 6300 1300 60  0001 C CNN
F 1 "GNDREF" H 6308 1369 60  0001 C CNN
F 2 "" H 6300 1550 60  0000 C CNN
F 3 "" H 6300 1550 60  0000 C CNN
	1    6300 1550
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR034
U 1 1 56CBC6C7
P 6900 1550
F 0 "#PWR034" H 6900 1300 60  0001 C CNN
F 1 "GNDREF" H 6908 1369 60  0001 C CNN
F 2 "" H 6900 1550 60  0000 C CNN
F 3 "" H 6900 1550 60  0000 C CNN
	1    6900 1550
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR035
U 1 1 56CBC7D2
P 7500 1550
F 0 "#PWR035" H 7500 1300 60  0001 C CNN
F 1 "GNDREF" H 7508 1369 60  0001 C CNN
F 2 "" H 7500 1550 60  0000 C CNN
F 3 "" H 7500 1550 60  0000 C CNN
	1    7500 1550
	1    0    0    -1  
$EndComp
Wire Notes Line
	8250 700  8250 2600
$Comp
L LM2931AZ-3.3/5.0 U2
U 1 1 56CBD242
P 6900 1000
F 0 "U2" H 6900 1368 40  0000 C CNN
F 1 "LM2931AZ-3.3/5.0" H 6900 1290 40  0000 C CNN
F 2 "Housings_TO-92:TO-92-Free-molded-wide" H 6900 1216 35  0000 C CIN
F 3 "" H 6900 1000 60  0000 C CNN
	1    6900 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 4550 9850 4550
Wire Wire Line
	8550 4550 8450 4650
Wire Wire Line
	8450 4650 7550 4650
Wire Wire Line
	8550 4650 9850 4650
Wire Wire Line
	8550 4650 8450 4550
Wire Wire Line
	8450 4550 7550 4550
$EndSCHEMATC
