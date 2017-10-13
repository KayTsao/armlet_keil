call "../../../hex2bin.exe" _build/nrf52832_xxaa_s132.hex d:/nrf52832_armlet.bin
c:
cd \Program Files (x86)\Nordic Semiconductor\Master Control Panel\3.10.0.14\nrf
call "nrfutil.exe" dfu genpkg --application "d:/nrf52832_armlet.bin" --application-version 0x00000001 --dev-revision 0x0001 --dev-type 0x0001 --sd-req 0xFFFE "d:/nrf52832_armlet.zip"