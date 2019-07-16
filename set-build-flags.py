from platform import system
Import("env")
# please keep $SOURCE variable, it will be replaced with a path to firmware
print("edited build_flags")
# print("Before")
# print(str(env['CPPDEFINES']))

env['CPPDEFINES'].remove(('VECT_TAB_ADDR', '0x8000000'))
# env['CPPDEFINES'].remove(('SERIAL_USB',))
env['CPPDEFINES'].remove(('CONFIG_MAPLE_MINI_NO_DISABLE_DEBUG', 1))
# env['CPPDEFINES'].remove(('ERROR_LED_PIN', 12)) // for Gogo Birght

env['CPPDEFINES'].append(('GENERIC_BOOTLOADER',))
env['CPPDEFINES'].append(('VECT_TAB_ADDR', '0x8000800'))
env['CPPDEFINES'].append(('ERROR_LED_PIN', 13))
# print("After")
print(str(env['CPPDEFINES']))

env.Replace(
    UPLOADER="hid-flash.exe" if system() == "Windows" else "hid-flash",
    UPLOADCMD='$UPLOADER $SOURCE "COM3"'
) 