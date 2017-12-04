# Robotiks WS17/17

# Assignment 6


| Name | MatrNr | Mail |
|------|----------|-----|
|Rémi Toudic | 4318284 | remitoudic@gmail.com|
| Sven Heinrichsen | 4780388| s.heinrichsen@fu-berlin.de |
| Alexander Hinze-Huettl | 4578322 | hinze.alex@gmail.com |

__Repo:__ [https://github.com/al-eax/robotik_ws1718](https://github.com/al-eax/robotik_ws1718)


## 2.​ ​Lane​ ​segmentation

The YUV color space seems to work best. This is because it can include every color for a specific white value. HSV works similarly but it is hard to find the right saturation value so that the color share stays low enough. RGB does not work well because the gray value correlates directly with the color shares.