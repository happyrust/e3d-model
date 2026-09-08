"""Decode PDMS/E3D noun and attribute hash codes back to names.

    hash = 27**4 + sum (ord(c_i) - 64) * 27**i     # i from 0, first letter lowest
"""

import sys

BASE = 27**4


def decode(code):
    v = code - BASE
    if v < 0:
        return None
    out = []
    while v > 0:
        d = v % 27
        if d == 0:
            out.append("?")
        else:
            out.append(chr(64 + d))
        v //= 27
    return "".join(out) or "?"


CRDESI = [
    718014, 785957, 703003, 912101, 640470, 818033, 956116, 837419, 549344,
    1014836, 779182, 790729, 636109, 832210, 564937, 904406, 897213, 841361,
    897228, 841766, 1026046, 599765, 560322, 959395, 822719, 985369, 207879217,
    621502, 621505, 719964, 719975, 813985, 900968, 900977, 914751, 783446,
    837961, 842877, 968612, 77028748, 128856299,
]

if __name__ == "__main__":
    # base 0：switch 的 case 立即数在伪码里是十六进制（`case 0x320775`），
    # 手工转十进制转错一位就会解出一个看着像 noun 的假名字。
    args = [int(a, 0) for a in sys.argv[1:]] or CRDESI
    for c in args:
        print("%-12d %s" % (c, decode(c)))
