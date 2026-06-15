# Load all symbol files (ROM ELF, bootloader, app)
source /home/boschi/Desktop/Mega/ESP32 - Enviromental Data Acquisition Board/ESP32 MQTT Acquisition Board/build/gdbinit/symbols

# Connect and halt both cores
set remotetimeout 10
target remote :3333
monitor reset halt

# Halt BOTH cores whenever a breakpoint is hit on either one
monitor esp smpbreak on

maintenance flush register-cache

# Break at app_main on first run so you can set further breakpoints before continuing
thbreak app_main
continue
