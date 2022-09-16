# Projekt stm32f4-asm-qemu

Osnovni projekt CubeIDE v zbirniku za stm32f4.

## Delovanje

Vsake pol sekunde (500 ms) LED diode spremenijo svoje stanje.
Interval se meri s časovnikom, proži se prekinitev na vsako ms in šteje do 500.

## Zagon

Projekt lahko pripravimo/zaženemo na 2 načina :
-  na sami plošči (Flash pomnilnik)
	- stm32f4-asm-BOARD.launch
		- desni klik na datoteko in Debug As
		
-  v simulatorju (Qemu xPack)
	- stm32f4-asm-QEMU.launch
		- desni klik na datoteko in Debug As
		
## Osnova

Narejeno na osnovi tega projekta
 https://github.com/fcayci/stm32f4-assembly`

