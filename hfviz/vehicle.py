'''
This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
'''


def get_vehicle(width, depth, length):

    # creates constants
    length = width

    # arm constants
    armLength = width*2
    armWidth = armLength/10

    # arrow constants
    arrowWidth = 1.0 * armWidth
    arrowLength = 5.0 * armWidth
    arrowHeight = 1.5 * depth

    # prop constants
    propWidth = 1.00 * armWidth
    propNarrowWidth = 0.20 * propWidth
    propLength = 7.50 * propWidth
    propNarrowLength = 0.75 * propLength
    propShortLength = 0.25 * propLength

    # prop pitch constants
    tipTU = 0.900 * depth
    tipTL = 0.625 * depth
    tipBU = 0.625 * depth
    tipBL = 0.350 * depth

    endT = .75 * depth
    endB = .50 * depth

    constant1 = ((endT-tipTL)/3) * depth
    constant2 = ((endB-tipBL)/3) * depth

    farTU = tipTU - constant2
    farTL = tipTL + constant1
    farBU = tipBU - constant1
    farBL = tipBL + constant2

    closeTU = farTU - constant2
    closeTL = farTL + constant1
    closeBU = farBU - constant1
    closeBL = farBL + constant2

    points = [

            # creates arm 1
            [+width - armWidth, +depth/2, +length + armWidth],  # 0   0
            [+width + armWidth, +depth/2, +length - armWidth],  # 1   1
            [+width + armWidth, -depth/2, +length - armWidth],  # 2   2
            [+width - armWidth, -depth/2, +length + armWidth],  # 3   3

            [+width + armLength - armWidth, +depth/2,
             +length + armLength + armWidth],                   # 4   4
            [+width + armLength + armWidth, +depth/2,
             +length + armLength - armWidth],                   # 5   5
            [+width + armLength + armWidth, -depth/2,
             +length + armLength - armWidth],                   # 6   6
            [+width + armLength - armWidth, -depth/2,
             +length + armLength + armWidth],                   # 7   7

            # creates arm 2
            [-width - armWidth, +depth/2, +length - armWidth],  # 0   8
            [-width + armWidth, +depth/2, +length + armWidth],  # 1   9
            [-width + armWidth, -depth/2, +length + armWidth],  # 2   10
            [-width - armWidth, -depth/2, +length - armWidth],  # 3   11

            [-width - armLength - armWidth, +depth/2,
             +length + armLength - armWidth],                   # 4   12
            [-width - armLength + armWidth, +depth/2,
             +length + armLength + armWidth],                   # 5   13
            [-width - armLength + armWidth, -depth/2,
             +length + armLength + armWidth],                   # 6   14
            [-width - armLength - armWidth, -depth/2,
             +length + armLength - armWidth],                   # 7   15

            # creates arm 3
            [+width + armLength - armWidth, +depth/2,
             -length - armLength - armWidth],                   # 0   16
            [+width + armLength + armWidth, +depth/2,
             -length - armLength + armWidth],                   # 1   17
            [+width + armLength + armWidth, -depth/2,
             -length - armLength + armWidth],                   # 2   18
            [+width + armLength - armWidth, -depth/2,
             -length - armLength - armWidth],                   # 3   19

            [+width - armWidth, +depth/2, -length - armWidth],  # 4   20
            [+width + armWidth, +depth/2, -length + armWidth],  # 5   21
            [+width + armWidth, -depth/2, -length + armWidth],  # 6   22
            [+width - armWidth, -depth/2, -length - armWidth],  # 7   23

            # creates arm 4
            [-width - armLength - armWidth, +depth/2,
             -length - armLength + armWidth],                   # 0   24
            [-width - armLength + armWidth, +depth/2,
             -length - armLength - armWidth],                   # 1   25
            [-width - armLength + armWidth, -depth/2,
             -length - armLength - armWidth],                   # 2   26
            [-width - armLength - armWidth, -depth/2,
             -length - armLength + armWidth],                   # 3   27

            [-width - armWidth, +depth/2, -length + armWidth],  # 4   28
            [-width + armWidth, +depth/2, -length - armWidth],  # 5   29
            [-width + armWidth, -depth/2, -length - armWidth],  # 6   30
            [-width - armWidth, -depth/2, -length + armWidth],  # 7   31

            # creates the arrow body
            [-arrowWidth, +arrowHeight, 0],                     # 0   32
            [+arrowWidth, +arrowHeight, 0],                     # 1   33
            [+arrowWidth, +depth, 0],                           # 2   34
            [-arrowWidth, +depth, 0],                           # 3   35

            [-arrowWidth, +arrowHeight, +arrowLength],          # 4  36
            [+arrowWidth, +arrowHeight, +arrowLength],          # 5  37
            [+arrowWidth, +depth, +arrowLength],                # 6  38
            [-arrowWidth, +depth, +arrowLength],                # 7  39

            # creates the arrow head
            [-(1/6)*arrowWidth, +arrowHeight, -arrowLength],    # 0  40
            [+(1/6)*arrowWidth, +arrowHeight, -arrowLength],    # 1  41
            [+(1/6)*arrowWidth, +depth, -arrowLength],          # 2  42
            [-(1/6)*arrowWidth, +depth, -arrowLength],          # 3  43

            [-arrowWidth - 2*arrowWidth, +arrowHeight, 0],      # 4  44
            [+arrowWidth + 2*arrowWidth, +arrowHeight, 0],      # 5  45
            [+arrowWidth + 2*arrowWidth, +depth, 0],            # 6  46
            [-arrowWidth - 2*arrowWidth, +depth, 0],            # 7  47

            # creates the center box
            [-width - armWidth, +depth, -length + armWidth],    # 0  48
            [-width + armWidth, +depth, -length - armWidth],    # 1  49

            [+width - armWidth, +depth, -length - armWidth],    # 2  50
            [+width + armWidth, +depth, -length + armWidth],    # 3  51

            [+width - armWidth, -depth, -length - armWidth],    # 4  52
            [+width + armWidth, -depth, -length + armWidth],    # 5  53

            [-width - armWidth, -depth, -length + armWidth],    # 6  54
            [-width + armWidth, -depth, -length - armWidth],    # 7  55

            [-width - armWidth, +depth, +length - armWidth],    # 8  56
            [-width + armWidth, +depth, +length + armWidth],    # 9  57

            [+width - armWidth, +depth, +length + armWidth],    # 10 58
            [+width + armWidth, +depth, +length - armWidth],    # 11 59

            [+width - armWidth, -depth, +length + armWidth],    # 12 60
            [+width + armWidth, -depth, +length - armWidth],    # 13 61

            [-width - armWidth, -depth, +length - armWidth],    # 14 62
            [-width + armWidth, -depth, +length + armWidth],    # 15 63

            # creates prop 1 on arm 1

            # North East far narrow tip
            [+width+armLength + propLength - propNarrowWidth, +tipTL,
             +length+armLength - propLength - propNarrowWidth],     # 0  64
            [+width+armLength + propLength + propNarrowWidth, +tipTU,
             +length+armLength - propLength + propNarrowWidth],     # 1  65
            [+width+armLength + propLength + propNarrowWidth, +tipBU,
             +length+armLength - propLength + propNarrowWidth],     # 2  66
            [+width+armLength + propLength - propNarrowWidth, +tipBL,
             +length+armLength - propLength - propNarrowWidth],     # 3  67

            # North East far wide
            [+width+armLength + propNarrowLength - propWidth, +farTL,
             +length+armLength - propNarrowLength - propWidth],     # 4  68
            [+width+armLength + propNarrowLength + propWidth, +farTU,
             +length+armLength - propNarrowLength + propWidth],     # 5  69
            [+width+armLength + propNarrowLength + propWidth, +farBU,
             +length+armLength - propNarrowLength + propWidth],     # 6  70
            [+width+armLength + propNarrowLength - propWidth, +farBL,
             +length+armLength - propNarrowLength - propWidth],     # 7  71

            # North East close wide
            [+width+armLength + propShortLength - propWidth, +closeTL,
             +length+armLength - propShortLength - propWidth],     # 4  72
            [+width+armLength + propShortLength + propWidth, +closeTU,
             +length+armLength - propShortLength + propWidth],     # 5  73
            [+width+armLength + propShortLength + propWidth, +farBU,
             +length+armLength - propShortLength + propWidth],       # 6  74
            [+width+armLength + propShortLength - propWidth, +farBL,
             +length+armLength - propShortLength - propWidth],       # 7  75

            # Middle narrow tip
            [+width+armLength - propNarrowWidth, +endT,
             +length+armLength - propNarrowWidth],    # 4  76
            [+width+armLength + propNarrowWidth, +endT,
             +length+armLength + propNarrowWidth],    # 5  77
            [+width+armLength + propNarrowWidth, +endB,
             +length+armLength + propNarrowWidth],    # 6  78
            [+width+armLength - propNarrowWidth, +endB,
             +length+armLength - propNarrowWidth],    # 7  79

            # South West close wide
            [+width+armLength - propShortLength - propWidth, +closeTU,
             +length+armLength + propShortLength - propWidth],     # 4  80
            [+width+armLength - propShortLength + propWidth, +closeTL,
             +length+armLength + propShortLength + propWidth],     # 5  81
            [+width+armLength - propShortLength + propWidth, +closeBL,
             +length+armLength + propShortLength + propWidth],     # 6  82
            [+width+armLength - propShortLength - propWidth, +closeBU,
             +length+armLength + propShortLength - propWidth],     # 7  83

            # South West far wide
            [+width+armLength - propNarrowLength - propWidth, +farTU,
             +length+armLength + propNarrowLength - propWidth],     # 4  84
            [+width+armLength - propNarrowLength + propWidth, +farTL,
             +length+armLength + propNarrowLength + propWidth],     # 5  85
            [+width+armLength - propNarrowLength + propWidth, +farBL,
             +length+armLength + propNarrowLength + propWidth],     # 6  86
            [+width+armLength - propNarrowLength - propWidth, +farBU,
             +length+armLength + propNarrowLength - propWidth],     # 7  87

            # South West far narrow tip
            [+width+armLength - propLength - propNarrowWidth, +tipTU,
             +length+armLength + propLength - propNarrowWidth],     # 0  88
            [+width+armLength - propLength + propNarrowWidth, +tipTL,
             +length+armLength + propLength + propNarrowWidth],     # 1  89
            [+width+armLength - propLength + propNarrowWidth, +tipBL,
             +length+armLength + propLength + propNarrowWidth],     # 2  90
            [+width+armLength - propLength - propNarrowWidth, +tipBU,
             +length+armLength + propLength - propNarrowWidth],     # 3  91

            # creates prop 4 on arm 4

            # North East far narrow tip
            [-width-armLength + propLength - propNarrowWidth, +tipTL,
             -length-armLength - propLength - propNarrowWidth],     # 0  92
            [-width-armLength + propLength + propNarrowWidth, +tipTU,
             -length-armLength - propLength + propNarrowWidth],     # 1  93
            [-width-armLength + propLength + propNarrowWidth, +tipBU,
             -length-armLength - propLength + propNarrowWidth],     # 2  94
            [-width-armLength + propLength - propNarrowWidth, +tipBL,
             -length-armLength - propLength - propNarrowWidth],     # 3  95

            # North East far wide
            [-width-armLength + propNarrowLength - propWidth, +farTL,
             -length-armLength - propNarrowLength - propWidth],     # 4  96
            [-width-armLength + propNarrowLength + propWidth, +farTU,
             -length-armLength - propNarrowLength + propWidth],     # 5  97
            [-width-armLength + propNarrowLength + propWidth, +farBU,
             -length-armLength - propNarrowLength + propWidth],     # 6  98
            [-width-armLength + propNarrowLength - propWidth, +farBL,
             -length-armLength - propNarrowLength - propWidth],     # 7  99

            # North East close wide
            [-width-armLength + propShortLength - propWidth, +closeTL,
             -length-armLength - propShortLength - propWidth],     # 4  100
            [-width-armLength + propShortLength + propWidth, +closeTU,
             -length-armLength - propShortLength + propWidth],     # 5  101
            [-width-armLength + propShortLength + propWidth, +closeBU,
             -length-armLength - propShortLength + propWidth],     # 6  102
            [-width-armLength + propShortLength - propWidth, +closeBL,
             -length-armLength - propShortLength - propWidth],     # 7  103

            # Middle narrow tip
            [-width-armLength - propNarrowWidth, +endT,
             -length-armLength - propNarrowWidth],    # 4  104
            [-width-armLength + propNarrowWidth, +endT,
             -length-armLength + propNarrowWidth],    # 5  105
            [-width-armLength + propNarrowWidth, +endB,
             -length-armLength + propNarrowWidth],    # 6  106
            [-width-armLength - propNarrowWidth, +endB,
             -length-armLength - propNarrowWidth],    # 7  107

            # South West close wide
            [-width-armLength - propShortLength - propWidth, +closeTU,
             -length-armLength + propShortLength - propWidth],     # 4  108
            [-width-armLength - propShortLength + propWidth, +closeTL,
             -length-armLength + propShortLength + propWidth],     # 5  109
            [-width-armLength - propShortLength + propWidth, +closeBL,
             -length-armLength + propShortLength + propWidth],     # 6  110
            [-width-armLength - propShortLength - propWidth, +closeBU,
             -length-armLength + propShortLength - propWidth],     # 7  111

            # South West far wide
            [-width-armLength - propNarrowLength - propWidth, +farTU,
             -length-armLength + propNarrowLength - propWidth],     # 4  112
            [-width-armLength - propNarrowLength + propWidth, +farTL,
             -length-armLength + propNarrowLength + propWidth],     # 5  113
            [-width-armLength - propNarrowLength + propWidth, +farBL,
             -length-armLength + propNarrowLength + propWidth],     # 6  114
            [-width-armLength - propNarrowLength - propWidth, +farBU,
             -length-armLength + propNarrowLength - propWidth],     # 7  115

            # South West far narrow tip
            [-width-armLength - propLength - propNarrowWidth, +tipTU,
             -length-armLength + propLength - propNarrowWidth],     # 0  116
            [-width-armLength - propLength + propNarrowWidth, +tipTL,
             -length-armLength + propLength + propNarrowWidth],     # 1  117
            [-width-armLength - propLength + propNarrowWidth, +tipBL,
             -length-armLength + propLength + propNarrowWidth],     # 2  118
            [-width-armLength - propLength - propNarrowWidth, +tipBU,
             -length-armLength + propLength - propNarrowWidth],     # 3  119

            # creates prop 3 on arm 3

            # North West far narrow tip
            [+width+armLength - propLength - propNarrowWidth, +tipTU,
             -length-armLength - propLength + propNarrowWidth],     # 0  120
            [+width+armLength - propLength + propNarrowWidth, +tipTL,
             -length-armLength - propLength - propNarrowWidth],     # 1  121
            [+width+armLength - propLength + propNarrowWidth, +tipBL,
             -length-armLength - propLength - propNarrowWidth],     # 2  122
            [+width+armLength - propLength - propNarrowWidth, +tipBU,
             -length-armLength - propLength + propNarrowWidth],     # 3  123

            # North West far wide
            [+width+armLength - propNarrowLength - propWidth, +farTU,
             -length-armLength - propNarrowLength + propWidth],     # 4  124
            [+width+armLength - propNarrowLength + propWidth, +farTL,
             -length-armLength - propNarrowLength - propWidth],     # 5  125
            [+width+armLength - propNarrowLength + propWidth, +farBL,
             -length-armLength - propNarrowLength - propWidth],     # 6  126
            [+width+armLength - propNarrowLength - propWidth, +farBU,
             -length-armLength - propNarrowLength + propWidth],     # 7  127

            # North West close wide
            [+width+armLength - propShortLength - propWidth, +closeTU,
             -length-armLength - propShortLength + propWidth],     # 4  128
            [+width+armLength - propShortLength + propWidth, +closeTL,
             -length-armLength - propShortLength - propWidth],     # 5  129
            [+width+armLength - propShortLength + propWidth, +closeBL,
             -length-armLength - propShortLength - propWidth],     # 6  130
            [+width+armLength - propShortLength - propWidth, +closeBU,
             -length-armLength - propShortLength + propWidth],     # 7  131

            # Middle narrow tip
            [+width+armLength - propNarrowWidth, +endT,
             -length-armLength + propNarrowWidth],                 # 4  132
            [+width+armLength + propNarrowWidth, +endT,
             -length-armLength - propNarrowWidth],                 # 5  133
            [+width+armLength + propNarrowWidth, +endB,
             -length-armLength - propNarrowWidth],                 # 6  134
            [+width+armLength - propNarrowWidth, +endB,
             -length-armLength + propNarrowWidth],                 # 7  135

            # South East close wide
            [+width+armLength + propShortLength - propWidth, +closeTL,
             -length-armLength + propShortLength + propWidth],     # 4  136
            [+width+armLength + propShortLength + propWidth, +closeTU,
             -length-armLength + propShortLength - propWidth],     # 5  137
            [+width+armLength + propShortLength + propWidth, +closeBU,
             -length-armLength + propShortLength - propWidth],     # 6  138
            [+width+armLength + propShortLength - propWidth, +closeBL,
             -length-armLength + propShortLength + propWidth],     # 7  139

            # South East far wide
            [+width+armLength + propNarrowLength - propWidth, +farTL,
             -length-armLength + propNarrowLength + propWidth],     # 4  140
            [+width+armLength + propNarrowLength + propWidth, +farTU,
             -length-armLength + propNarrowLength - propWidth],     # 5  141
            [+width+armLength + propNarrowLength + propWidth, +farBU,
             -length-armLength + propNarrowLength - propWidth],     # 6  142
            [+width+armLength + propNarrowLength - propWidth, +farBL,
             -length-armLength + propNarrowLength + propWidth],     # 7  143

            # South East far narrow tip
            [+width+armLength + propLength - propNarrowWidth, +tipTL,
             -length-armLength + propLength + propNarrowWidth],     # 0  144
            [+width+armLength + propLength + propNarrowWidth, +tipTU,
             -length-armLength + propLength - propNarrowWidth],     # 1  145
            [+width+armLength + propLength + propNarrowWidth, +tipBU,
             -length-armLength + propLength - propNarrowWidth],     # 2  146
            [+width+armLength + propLength - propNarrowWidth, +tipBL,
             -length-armLength + propLength + propNarrowWidth],     # 3  147

            # creates prop 2 on arm 2

            # North West far narrow tip
            [-width-armLength - propLength - propNarrowWidth, +tipTU,
             +length+armLength - propLength + propNarrowWidth],     # 0  148
            [-width-armLength - propLength + propNarrowWidth, +tipTL,
             +length+armLength - propLength - propNarrowWidth],     # 1  149
            [-width-armLength - propLength + propNarrowWidth, +tipBL,
             +length+armLength - propLength - propNarrowWidth],     # 2  150
            [-width-armLength - propLength - propNarrowWidth, +tipBU,
             +length+armLength - propLength + propNarrowWidth],     # 3  151

            # North West far wide
            [-width-armLength - propNarrowLength - propWidth, +farTU,
             +length+armLength - propNarrowLength + propWidth],     # 4  152
            [-width-armLength - propNarrowLength + propWidth, +farTL,
             +length+armLength - propNarrowLength - propWidth],     # 5  153
            [-width-armLength - propNarrowLength + propWidth, +farBL,
             +length+armLength - propNarrowLength - propWidth],     # 6  154
            [-width-armLength - propNarrowLength - propWidth, +farBU,
             +length+armLength - propNarrowLength + propWidth],     # 7  155

            # North West close wide
            [-width-armLength - propShortLength - propWidth, +closeTU,
             +length+armLength - propShortLength + propWidth],     # 4  156
            [-width-armLength - propShortLength + propWidth, +closeTL,
             +length+armLength - propShortLength - propWidth],     # 5  157
            [-width-armLength - propShortLength + propWidth, +closeBL,
             +length+armLength - propShortLength - propWidth],     # 6  158
            [-width-armLength - propShortLength - propWidth, +closeBU,
             +length+armLength - propShortLength + propWidth],     # 7  159

            # Middle narrow tip
            [-width-armLength - propNarrowWidth, +endT,
             +length+armLength + propNarrowWidth],                 # 4  160
            [-width-armLength + propNarrowWidth, +endT,
             +length+armLength - propNarrowWidth],                 # 5  161
            [-width-armLength + propNarrowWidth, +endB,
             +length+armLength - propNarrowWidth],                 # 6  162
            [-width-armLength - propNarrowWidth, +endB,
             +length+armLength + propNarrowWidth],                 # 7  163

            # South East close wide
            [-width-armLength + propShortLength - propWidth, +closeTL,
             +length+armLength + propShortLength + propWidth],     # 4  164
            [-width-armLength + propShortLength + propWidth, +closeTU,
             +length+armLength + propShortLength - propWidth],     # 5  165
            [-width-armLength + propShortLength + propWidth, +closeBU,
             +length+armLength + propShortLength - propWidth],     # 6  166
            [-width-armLength + propShortLength - propWidth, +closeBL,
             +length+armLength + propShortLength + propWidth],     # 7  167

            # South East far wide
            [-width-armLength + propNarrowLength - propWidth, +farTL,
             +length+armLength + propNarrowLength + propWidth],     # 4  168
            [-width-armLength + propNarrowLength + propWidth, +farTU,
             +length+armLength + propNarrowLength - propWidth],     # 5  169
            [-width-armLength + propNarrowLength + propWidth, +farBU,
             +length+armLength + propNarrowLength - propWidth],     # 6  170
            [-width-armLength + propNarrowLength - propWidth, +farBL,
             +length+armLength + propNarrowLength + propWidth],     # 7  171

            # South East far narrow tip
            [-width-armLength + propLength - propNarrowWidth, +tipTL,
             +length+armLength + propLength + propNarrowWidth],     # 0   172
            [-width-armLength + propLength + propNarrowWidth, +tipTU,
             +length+armLength + propLength - propNarrowWidth],     # 1   173
            [-width-armLength + propLength + propNarrowWidth, +tipBU,
             +length+armLength + propLength - propNarrowWidth],     # 2  174
            [-width-armLength + propLength - propNarrowWidth, +tipBL,
             +length+armLength + propLength + propNarrowWidth]      # 3  175
            ]

    #  Each face contains indices into points array above
    faces = [
             # top of the Box
             (50, 49, 48, 51), (59, 51, 48, 56), (58, 59, 56, 57),

             # arrow Head
             (40, 41, 42, 43), (41, 45, 46, 42), (45, 44, 47, 46),
             (44, 40, 43, 47), (40, 44, 45, 41), (43, 42, 46, 47),

             # arrow Body
             (32, 33, 34, 35), (33, 37, 38, 34), (37, 36, 39, 38),
             (36, 32, 35, 39), (32, 36, 37, 33), (35, 34, 38, 39),

             # rest of the box
             (54, 55, 52, 53), (54, 53, 61, 62), (62, 61, 60, 63),
             (48, 49, 55, 54), (49, 50, 52, 55), (50, 51, 53, 52),
             (51, 59, 61, 53), (59, 58, 60, 61), (58, 57, 63, 60),
             (57, 56, 62, 63), (56, 48, 54, 62),

             # arm 1
             (1, 5, 6, 2), (5, 4, 7, 6), (4, 0, 3, 7), (0, 4, 5, 1),
             (3, 2, 6, 7),

             # arm2
             (9, 13, 14, 10), (13, 12, 15, 14), (12, 8, 11, 15),
             (8, 12, 13, 9), (11, 10, 14, 15),

             # arm3
             (16, 17, 18, 19), (17, 21, 22, 18), (20, 16, 19, 23),
             (16, 20, 21, 17), (19, 18, 22, 23),

             # arm4
             (24, 25, 26, 27), (25, 29, 30, 26), (28, 24, 27, 31),
             (24, 28, 29, 25), (27, 26, 30, 31),

             # prop 4
             (92, 93, 94, 95), (93, 97, 98, 94), (97, 96, 99, 98),
             (96, 92, 95, 99), (92, 96, 97, 93), (95, 94, 98, 99),
             (97, 101, 102, 98), (101, 100, 103, 102), (100, 96, 99, 103),
             (96, 100, 101, 97), (99, 98, 102, 103), (101, 105, 106, 102),
             (104, 100, 103, 107), (100, 104, 105, 101), (103, 102, 106, 107),
             (105, 109, 110, 106), (108, 104, 107, 111), (104, 108, 109, 105),
             (107, 106, 110, 111), (109, 113, 114, 110), (112, 108, 111, 115),
             (108, 112, 113, 109), (111, 110, 114, 115), (113, 117, 118, 114),
             (117, 116, 119, 118), (116, 112, 115, 119), (112, 116, 117, 113),
             (115, 114, 118, 119),

             # prop3
             (120, 121, 122, 123), (121, 125, 126, 122), (124, 120, 123, 127),
             (120, 124, 125, 121), (123, 122, 126, 127), (125, 129, 130, 126),
             (128, 124, 127, 131), (124, 128, 129, 125), (127, 126, 130, 131),
             (129, 133, 134, 130), (132, 128, 131, 135), (128, 132, 133, 129),
             (131, 130, 134, 135), (133, 137, 138, 134), (136, 132, 135, 139),
             (132, 136, 137, 133), (135, 134, 138, 139), (137, 141, 142, 138),
             (140, 136, 139, 143), (136, 140, 141, 137), (139, 138, 142, 143),
             (141, 145, 146, 142), (145, 144, 147, 146), (144, 140, 143, 147),
             (140, 144, 145, 141), (143, 142, 146, 147),

             # prop2
             (148, 149, 150, 151), (149, 153, 154, 150), (152, 148, 151, 155),
             (148, 152, 153, 149), (151, 150, 154, 155), (153, 157, 158, 154),
             (156, 152, 155, 159), (152, 156, 157, 153), (155, 154, 158, 159),
             (157, 161, 162, 158), (160, 156, 159, 163), (156, 160, 161, 157),
             (159, 158, 162, 163), (161, 165, 166, 162), (164, 160, 163, 167),
             (160, 164, 165, 161), (163, 162, 166, 167), (165, 169, 170, 166),
             (168, 164, 167, 171), (164, 168, 169, 165), (167, 166, 170, 171),
             (169, 173, 174, 170), (173, 172, 175, 174), (172, 168, 171, 175),
             (168, 172, 173, 169), (171, 170, 174, 175),

             # prop1
             (64, 65, 66, 67), (65, 69, 70, 66), (68, 64, 67, 71),
             (64, 68, 69, 65), (67, 66, 70, 71), (69, 73, 74, 70),
             (72, 68, 71, 75), (68, 72, 73, 69), (71, 70, 74, 75),
             (73, 77, 78, 74), (76, 72, 75, 79), (72, 76, 77, 73),
             (75, 74, 78, 79), (77, 81, 82, 78), (80, 76, 79, 83),
             (76, 80, 81, 77), (79, 78, 82, 83), (81, 85, 86, 82),
             (84, 80, 83, 87), (80, 84, 85, 81), (83, 82, 86, 87),
             (85, 89, 90, 86), (89, 88, 91, 90), (88, 84, 87, 91),
             (84, 88, 89, 85), (87, 86, 90, 91),
             ]

    lightGrey = '#72716d'
    grey = '#665f59'
    darkGrey = '#4c4641'
    darkRed = '#993838'
    red = 'red'
    green = '#31e224'
    darkGreen = '#2b7f24'

    colors = [
              lightGrey, lightGrey, lightGrey,                  # box Top

              'red', 'red', 'red', 'red', 'red', 'red',         # arrow head
              'red', 'red', 'red', 'red', 'red', 'red',         # arrow body

              lightGrey, lightGrey, lightGrey,                  # box bottom
              grey, grey, grey,                                 # box North
              grey,                                             # box East
              grey, grey, grey,                                 # box South
              grey,                                             # box West

              lightGrey, grey, lightGrey, darkGrey, darkGrey,   # arm 1
              lightGrey, grey, lightGrey, darkGrey, darkGrey,   # arm 2
              grey, lightGrey, lightGrey, darkGrey, darkGrey,   # arm 3
              grey, lightGrey, lightGrey, darkGrey, darkGrey,   # arm 4

              # prop 4
              darkGreen, darkGreen, darkGreen, darkGreen, green,
              green, darkGreen, darkGreen, darkGreen,
              green, green, darkGreen, darkGreen,
              green, green, darkGreen, darkGreen,
              green, green, darkGreen,
              darkGreen, green, green, darkGreen,
              darkGreen, darkGreen, green, green,

              # prop 3
              darkGreen, darkGreen, darkGreen, green,
              green, darkGreen, darkGreen, green,
              green, darkGreen, darkGreen, green,
              green, darkGreen, darkGreen, green,
              green, darkGreen, darkGreen, green,
              green, darkGreen, darkGreen, darkGreen, green, green,

              # prop 2
              darkRed, darkRed, darkRed, red,
              red, darkRed, darkRed, red,
              red, darkRed, darkRed, red,
              red, darkRed, darkRed, red,
              red, darkRed, darkRed, red,
              red, darkRed, darkRed, darkRed, red, red,

              # prop 1
              darkRed, darkRed, darkRed, red, red,
              darkRed, darkRed, red, red,
              darkRed, darkRed, red, red,
              darkRed, darkRed, red,
              red, darkRed, darkRed, red,
              red, darkRed, darkRed, darkRed, red, red,
              ]
    return points, faces, colors
