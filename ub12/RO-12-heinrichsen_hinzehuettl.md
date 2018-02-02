# Robotiks WS17/18

# Assignment 12

| Name |   MatrNr | Mail |
|------|----------|-----|
| Sven Heinrichsen | 4780388 | s.heinrichsen@fu-berlin.de |
| Alexander Hinze-Huettl | 4578322 | hinze.alex@gmail.com |

__Repo:__ [https://github.com/al-eax/robotik_ws1718](https://github.com/al-eax/robotik_ws1718)

## A* Tree

|     Step   |         Open  {g+h}        |   Close      |
|------------|---------------------------------------------|---------------------- |
|0           |  C{10+42}, D{10+35}, E{10+20}, __G{10+1}__  | H{0+19}      |
|1           | C{10+42}, D{10+35}, __E{10+20}__, F{20+18}  | H{0+19}, G{10+1}     |
|2           | C{10+42}, D{10+35}, __F{20+18}__   | H{0+19}, G{10+1}, E{10+20}  |
|3           | C{10+42}, D{10+35}, __B{30+9}__     | H{0+19}, G{10+1}, E{10+20}, F{20+18}  |
|4           | C{10+42}, __D{10+35}__ , _A{40+0}_    | H{0+19}, G{10+1}, E{10+20}, F{20+18}, B{30+9}  |
|5           | __C{10+42}__ , _A{40+0}_, _A{20+0}_   | H{0+19}, G{10+1}, E{10+20}, F{20+18}, B{30+9}, D{10+35} |
|6           | _A{40+0}_, ___A{20+0}___, _A{20+0}_   | H{0+19}, G{10+1}, E{10+20}, F{20+18}, B{30+9}, D{10+35}, C{10+42} |

Shortest way under consideration of alphanumerical values: \(h \rightarrow c \rightarrow a\).
This heuristic is __not__ optimistic. Some heuristics are higher than the actual path costs.
