# P&G JC200 #

![http://pwct.googlecode.com/git/wiki/remoteplus.png](http://pwct.googlecode.com/git/wiki/remoteplus.png)

This joystick came from a PGDT/Pride 3-Key [Remote Plus Controller](http://www.pridemobility.com/pdf/owners_manuals/operation_instructions/qr_remote_plus_boi.pdf). Inside of this controller there is a P&G Controls [JC200](http://www.pennyandgiles.com/Multi-Axis-Inductive-Joystick-Controller-pd-104,3,,.php) inductive joystick ([datasheet](http://www.pennyandgiles.com/script_cms/force_file_download.php?fileID=103)).

**Wiring**
| **Wire color** | **Description** | **I or O** |
|:---------------|:----------------|:-----------|
| Red            | +12V            | Input      |
| Green          | Center tap      | Output     |
| Black          | GND             | GND        |
| Blue           | x-axis / direction | Output     |
| Yellow         | y-axis / speed  | Output     |

**Electrical characteristics**
| Nominal supply voltage | 12V |
|:-----------------------|:----|
| Current draw           | 16mA |
| Green center tap       | 5.99V |
| Forward                | yellow, voltage > 5.98V |
| Backward               | yellow, voltage < 5.98V |
| Right                  | blue, voltage > 5.97V |
| Left                   | blue, voltage < 5.97V |



# Invacare #

![http://pwct.googlecode.com/git/wiki/invacare.png](http://pwct.googlecode.com/git/wiki/invacare.png)

This joystick came from an Invacare MKIV A controller.

The joystick has a notch indicating backward direction.

![http://pwct.googlecode.com/git/wiki/invacare_direction.png](http://pwct.googlecode.com/git/wiki/invacare_direction.png)

**Wiring**
| **Wire color** | **Description** | **I or O** |
|:---------------|:----------------|:-----------|
| Red            | +5V             | Input      |
| Black          | GND             | GND        |
| Yellow         | Speed           | Output     |
| Blue           | Direction       | Output     |

**Electrical characteristics**
| Nominal supply voltage | 5V |
|:-----------------------|:---|
| Start-up delay         | 150μs |
| Current draw           | 9mA |
| Forward                | yellow, voltage > 2.51V |
| Backward               | yellow, voltage < 2.51V |
| Right                  | blue, voltage > 2.51V |
| Left                   | blue, voltage < 2.51V |