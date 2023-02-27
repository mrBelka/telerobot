### Инструкции
1. Настройки смены языка в Ubuntu 22.04
````
gsettings set org.gnome.desktop.wm.keybindings switch-input-source "['<Shift>Alt_L']"
gsettings set org.gnome.desktop.wm.keybindings switch-input-source-backward "['<Alt>Shift_L']"
````
2. /dev/ttyUSB0 не может появиться, т.к. идентификатор преобрзователя Arduino 
в Ubuntu 22.04 занят. Надо его отключить:

````
    1. Edit /usr/lib/udev/rules.d/85-brltty.rules
    2. Search for this line and comment it out:
       ENV{PRODUCT}=="1a86/7523/*", ENV{BRLTTY_BRAILLE_DRIVER}="bm", GOTO="brltty_usb_run"
    3. reboot
````

3. mbpoll
````
mbpoll -m rtu -b 9600 -P none -t4 -r 9 -v /dev/ttyUSB0 -- 2
mbpoll -m rtu -b 9600 -P none -t3 -r 8 /dev/ttyUSB0
````
