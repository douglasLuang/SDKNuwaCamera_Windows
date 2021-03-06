#include "TmpTable.h"

#if (c_ntc_100k == 1)
const int tempTab[101] = {
    12602,//50,//=ROUND(100000*EXP(4250*(1/(273.15-20)-1/(273.15+25))),0)
    11796,//92,//=ROUND(100000*EXP(4250*(1/(273.15-19)-1/(273.15+25))),0)
    11048,//54,//=ROUND(100000*EXP(4250*(1/(273.15-18)-1/(273.15+25))),0)
    10352,//94,//=ROUND(100000*EXP(4250*(1/(273.15-17)-1/(273.15+25))),0)
    9706,//04,//=ROUND(100000*EXP(4250*(1/(273.15-16)-1/(273.15+25))),0)
    9104,//12,//=ROUND(100000*EXP(4250*(1/(273.15-15)-1/(273.15+25))),0)
    8543,//74,//=ROUND(100000*EXP(4250*(1/(273.15-14)-1/(273.15+25))),0)
    8021,//77,//=ROUND(100000*EXP(4250*(1/(273.15-13)-1/(273.15+25))),0)
    7535,//33,//=ROUND(100000*EXP(4250*(1/(273.15-12)-1/(273.15+25))),0)
    7081,//76,//=ROUND(100000*EXP(4250*(1/(273.15-11)-1/(273.15+25))),0)
    6658,//64,//=ROUND(100000*EXP(4250*(1/(273.15-10)-1/(273.15+25))),0)
    6263,//71,//=ROUND(100000*EXP(4250*(1/(273.15- 9)-1/(273.15+25))),0)
    5894,//93,//=ROUND(100000*EXP(4250*(1/(273.15- 8)-1/(273.15+25))),0)
    5550,//39,//=ROUND(100000*EXP(4250*(1/(273.15- 7)-1/(273.15+25))),0)
    5228,//35,//=ROUND(100000*EXP(4250*(1/(273.15- 6)-1/(273.15+25))),0)
    4927,//19,//=ROUND(100000*EXP(4250*(1/(273.15- 5)-1/(273.15+25))),0)
    4645,//42,//=ROUND(100000*EXP(4250*(1/(273.15- 4)-1/(273.15+25))),0)
    4381,//67,//=ROUND(100000*EXP(4250*(1/(273.15- 3)-1/(273.15+25))),0)
    4134,//69,//=ROUND(100000*EXP(4250*(1/(273.15- 2)-1/(273.15+25))),0)
    3903,//28,//=ROUND(100000*EXP(4250*(1/(273.15- 1)-1/(273.15+25))),0)
    3686,//39,//=ROUND(100000*EXP(4250*(1/(273.15- 0)-1/(273.15+25))),0)
    3482,//99,//=ROUND(100000*EXP(4250*(1/(273.15+ 1)-1/(273.15+25))),0)
    3292,//18,//=ROUND(100000*EXP(4250*(1/(273.15+ 2)-1/(273.15+25))),0)
    3113,//09,//=ROUND(100000*EXP(4250*(1/(273.15+ 3)-1/(273.15+25))),0)
    2944,//93,//=ROUND(100000*EXP(4250*(1/(273.15+ 4)-1/(273.15+25))),0)
    2786,//97,//=ROUND(100000*EXP(4250*(1/(273.15+ 5)-1/(273.15+25))),0)
    2638,//52,//=ROUND(100000*EXP(4250*(1/(273.15+ 6)-1/(273.15+25))),0)
    2498,//96,//=ROUND(100000*EXP(4250*(1/(273.15+ 7)-1/(273.15+25))),0)
    2367,//69,//=ROUND(100000*EXP(4250*(1/(273.15+ 8)-1/(273.15+25))),0)
    2244,//18,//=ROUND(100000*EXP(4250*(1/(273.15+ 9)-1/(273.15+25))),0)
    2127,//91,//=ROUND(100000*EXP(4250*(1/(273.15+10)-1/(273.15+25))),0)
    2018,//43,//=ROUND(100000*EXP(4250*(1/(273.15+11)-1/(273.15+25))),0)
    1915,//28,//=ROUND(100000*EXP(4250*(1/(273.15+12)-1/(273.15+25))),0)
    1818,//08,//=ROUND(100000*EXP(4250*(1/(273.15+13)-1/(273.15+25))),0)
    1726,//43,//=ROUND(100000*EXP(4250*(1/(273.15+14)-1/(273.15+25))),0)
    1639,//99,//=ROUND(100000*EXP(4250*(1/(273.15+15)-1/(273.15+25))),0)
    1558,//44,//=ROUND(100000*EXP(4250*(1/(273.15+16)-1/(273.15+25))),0)
    1481,//46,//=ROUND(100000*EXP(4250*(1/(273.15+17)-1/(273.15+25))),0)
    1408,//77,//=ROUND(100000*EXP(4250*(1/(273.15+18)-1/(273.15+25))),0)
    1340,//11,//=ROUND(100000*EXP(4250*(1/(273.15+19)-1/(273.15+25))),0)
    1275,//23,//=ROUND(100000*EXP(4250*(1/(273.15+20)-1/(273.15+25))),0)
    1213,//90,//=ROUND(100000*EXP(4250*(1/(273.15+21)-1/(273.15+25))),0)
    1155,//91,//=ROUND(100000*EXP(4250*(1/(273.15+22)-1/(273.15+25))),0)
    1101,//05,//=ROUND(100000*EXP(4250*(1/(273.15+23)-1/(273.15+25))),0)
    1049,//14,//=ROUND(100000*EXP(4250*(1/(273.15+24)-1/(273.15+25))),0)
    1000,//00,//=ROUND(100000*EXP(4250*(1/(273.15+25)-1/(273.15+25))),0)
    953,//47,//=ROUND(100000*EXP(4250*(1/(273.15+26)-1/(273.15+25))),0)
    909,//39,//=ROUND(100000*EXP(4250*(1/(273.15+27)-1/(273.15+25))),0)
    867,//62,//=ROUND(100000*EXP(4250*(1/(273.15+28)-1/(273.15+25))),0)
    828,//03,//=ROUND(100000*EXP(4250*(1/(273.15+29)-1/(273.15+25))),0)
    790,//49,//=ROUND(100000*EXP(4250*(1/(273.15+30)-1/(273.15+25))),0)
    754,//88,//=ROUND(100000*EXP(4250*(1/(273.15+31)-1/(273.15+25))),0)
    721,//09,//=ROUND(100000*EXP(4250*(1/(273.15+32)-1/(273.15+25))),0)
    689,//02,//=ROUND(100000*EXP(4250*(1/(273.15+33)-1/(273.15+25))),0)
    658,//57,//=ROUND(100000*EXP(4250*(1/(273.15+34)-1/(273.15+25))),0)
    629,//65,//=ROUND(100000*EXP(4250*(1/(273.15+35)-1/(273.15+25))),0)
    602,//18,//=ROUND(100000*EXP(4250*(1/(273.15+36)-1/(273.15+25))),0)
    576,//07,//=ROUND(100000*EXP(4250*(1/(273.15+37)-1/(273.15+25))),0)
    551,//25,//=ROUND(100000*EXP(4250*(1/(273.15+38)-1/(273.15+25))),0)
    527,//65,//=ROUND(100000*EXP(4250*(1/(273.15+39)-1/(273.15+25))),0)
    505,//20,//=ROUND(100000*EXP(4250*(1/(273.15+40)-1/(273.15+25))),0)
    483,//84,//=ROUND(100000*EXP(4250*(1/(273.15+41)-1/(273.15+25))),0)
    463,//51,//=ROUND(100000*EXP(4250*(1/(273.15+42)-1/(273.15+25))),0)
    444,//15,//=ROUND(100000*EXP(4250*(1/(273.15+43)-1/(273.15+25))),0)
    425,//72,//=ROUND(100000*EXP(4250*(1/(273.15+44)-1/(273.15+25))),0)
    408,//16,//=ROUND(100000*EXP(4250*(1/(273.15+45)-1/(273.15+25))),0)
    391,//43,//=ROUND(100000*EXP(4250*(1/(273.15+46)-1/(273.15+25))),0)
    375,//48,//=ROUND(100000*EXP(4250*(1/(273.15+47)-1/(273.15+25))),0)
    360,//28,//=ROUND(100000*EXP(4250*(1/(273.15+48)-1/(273.15+25))),0)
    345,//78,//=ROUND(100000*EXP(4250*(1/(273.15+49)-1/(273.15+25))),0)
    331,//95,//=ROUND(100000*EXP(4250*(1/(273.15+50)-1/(273.15+25))),0)
    314,//23,//=ROUND(100000*EXP(4303*(1/(273.15+51)-1/(273.15+25))),0)
    301,//66,//=ROUND(100000*EXP(4303*(1/(273.15+52)-1/(273.15+25))),0)
    289,//67,//=ROUND(100000*EXP(4303*(1/(273.15+53)-1/(273.15+25))),0)
    278,//22,//=ROUND(100000*EXP(4303*(1/(273.15+54)-1/(273.15+25))),0)
    267,//29,//=ROUND(100000*EXP(4303*(1/(273.15+55)-1/(273.15+25))),0)
    256,//85,//=ROUND(100000*EXP(4303*(1/(273.15+56)-1/(273.15+25))),0)
    246,//88,//=ROUND(100000*EXP(4303*(1/(273.15+57)-1/(273.15+25))),0)
    237,//35,//=ROUND(100000*EXP(4303*(1/(273.15+58)-1/(273.15+25))),0)
    228,//24,//=ROUND(100000*EXP(4303*(1/(273.15+59)-1/(273.15+25))),0)
    219,//54,//=ROUND(100000*EXP(4303*(1/(273.15+60)-1/(273.15+25))),0)
    211,//21,//=ROUND(100000*EXP(4303*(1/(273.15+61)-1/(273.15+25))),0)
    203,//25,//=ROUND(100000*EXP(4303*(1/(273.15+62)-1/(273.15+25))),0)
    195,//64,//=ROUND(100000*EXP(4303*(1/(273.15+63)-1/(273.15+25))),0)
    188,//35,//=ROUND(100000*EXP(4303*(1/(273.15+64)-1/(273.15+25))),0)
    181,//37,//=ROUND(100000*EXP(4303*(1/(273.15+65)-1/(273.15+25))),0)
    174,//69,//=ROUND(100000*EXP(4303*(1/(273.15+66)-1/(273.15+25))),0)
    168,//30,//=ROUND(100000*EXP(4303*(1/(273.15+67)-1/(273.15+25))),0)
    162,//17,//=ROUND(100000*EXP(4303*(1/(273.15+68)-1/(273.15+25))),0)
    156,//30,//=ROUND(100000*EXP(4303*(1/(273.15+69)-1/(273.15+25))),0)
    150,//68,//=ROUND(100000*EXP(4303*(1/(273.15+70)-1/(273.15+25))),0)
    145,//28,//=ROUND(100000*EXP(4303*(1/(273.15+71)-1/(273.15+25))),0)
    140,//12,//=ROUND(100000*EXP(4303*(1/(273.15+72)-1/(273.15+25))),0)
    135,//16,//=ROUND(100000*EXP(4303*(1/(273.15+73)-1/(273.15+25))),0)
    130,//40,//=ROUND(100000*EXP(4303*(1/(273.15+74)-1/(273.15+25))),0)
    125,//84,//=ROUND(100000*EXP(4303*(1/(273.15+75)-1/(273.15+25))),0)
    121,//47,//=ROUND(100000*EXP(4303*(1/(273.15+76)-1/(273.15+25))),0)
    117,//27,//=ROUND(100000*EXP(4303*(1/(273.15+77)-1/(273.15+25))),0)
    113,//23,//=ROUND(100000*EXP(4303*(1/(273.15+78)-1/(273.15+25))),0)
    109,//36,//=ROUND(100000*EXP(4303*(1/(273.15+79)-1/(273.15+25))),0)
    105,//64,//=ROUND(100000*EXP(4303*(1/(273.15+80)-1/(273.15+25))),0)
};
#elif (c_ntc_10k == 1)
const int tempTab[101] = {
    7502,//2,//=ROUND(10000*EXP(3380*(1/(273.15-20)-1/(273.15+25))),0)
    7118,//2,//=ROUND(10000*EXP(3380*(1/(273.15-19)-1/(273.15+25))),0)
    6756,//7,//=ROUND(10000*EXP(3380*(1/(273.15-18)-1/(273.15+25))),0)
    6416,//1,//=ROUND(10000*EXP(3380*(1/(273.15-17)-1/(273.15+25))),0)
    6095,//2,//=ROUND(10000*EXP(3380*(1/(273.15-16)-1/(273.15+25))),0)
    5792,//6,//=ROUND(10000*EXP(3380*(1/(273.15-15)-1/(273.15+25))),0)
    5507,//2,//=ROUND(10000*EXP(3380*(1/(273.15-14)-1/(273.15+25))),0)
    5237,//9,//=ROUND(10000*EXP(3380*(1/(273.15-13)-1/(273.15+25))),0)
    4983,//7,//=ROUND(10000*EXP(3380*(1/(273.15-12)-1/(273.15+25))),0)
    4743,//6,//=ROUND(10000*EXP(3380*(1/(273.15-11)-1/(273.15+25))),0)
    4516,//8,//=ROUND(10000*EXP(3380*(1/(273.15-10)-1/(273.15+25))),0)
    4302,//4,//=ROUND(10000*EXP(3380*(1/(273.15- 9)-1/(273.15+25))),0)
    4099,//7,//=ROUND(10000*EXP(3380*(1/(273.15- 8)-1/(273.15+25))),0)
    3908,//0,//=ROUND(10000*EXP(3380*(1/(273.15- 7)-1/(273.15+25))),0)
    3726,//6,//=ROUND(10000*EXP(3380*(1/(273.15- 6)-1/(273.15+25))),0)
    3554,//8,//=ROUND(10000*EXP(3380*(1/(273.15- 5)-1/(273.15+25))),0)
    3392,//2,//=ROUND(10000*EXP(3380*(1/(273.15- 4)-1/(273.15+25))),0)
    3238,//1,//=ROUND(10000*EXP(3380*(1/(273.15- 3)-1/(273.15+25))),0)
    3092,//1,//=ROUND(10000*EXP(3380*(1/(273.15- 2)-1/(273.15+25))),0)
    2953,//7,//=ROUND(10000*EXP(3380*(1/(273.15- 1)-1/(273.15+25))),0)
    2822,//4,//=ROUND(10000*EXP(3380*(1/(273.15- 0)-1/(273.15+25))),0)
    2697,//8,//=ROUND(10000*EXP(3380*(1/(273.15+ 1)-1/(273.15+25))),0)
    2579,//6,//=ROUND(10000*EXP(3380*(1/(273.15+ 2)-1/(273.15+25))),0)
    2467,//4,//=ROUND(10000*EXP(3380*(1/(273.15+ 3)-1/(273.15+25))),0)
    2360,//8,//=ROUND(10000*EXP(3380*(1/(273.15+ 4)-1/(273.15+25))),0)
    2259,//5,//=ROUND(10000*EXP(3380*(1/(273.15+ 5)-1/(273.15+25))),0)
    2163,//2,//=ROUND(10000*EXP(3380*(1/(273.15+ 6)-1/(273.15+25))),0)
    2071,//7,//=ROUND(10000*EXP(3380*(1/(273.15+ 7)-1/(273.15+25))),0)
    1984,//7,//=ROUND(10000*EXP(3380*(1/(273.15+ 8)-1/(273.15+25))),0)
    1901,//9,//=ROUND(10000*EXP(3380*(1/(273.15+ 9)-1/(273.15+25))),0)
    1823,//1,//=ROUND(10000*EXP(3380*(1/(273.15+10)-1/(273.15+25))),0)
    1748,//1,//=ROUND(10000*EXP(3380*(1/(273.15+11)-1/(273.15+25))),0)
    1676,//7,//=ROUND(10000*EXP(3380*(1/(273.15+12)-1/(273.15+25))),0)
    1608,//7,//=ROUND(10000*EXP(3380*(1/(273.15+13)-1/(273.15+25))),0)
    1543,//8,//=ROUND(10000*EXP(3380*(1/(273.15+14)-1/(273.15+25))),0)
    1482,//0,//=ROUND(10000*EXP(3380*(1/(273.15+15)-1/(273.15+25))),0)
    1423,//1,//=ROUND(10000*EXP(3380*(1/(273.15+16)-1/(273.15+25))),0)
    1366,//9,//=ROUND(10000*EXP(3380*(1/(273.15+17)-1/(273.15+25))),0)
    1313,//3,//=ROUND(10000*EXP(3380*(1/(273.15+18)-1/(273.15+25))),0)
    1262,//2,//=ROUND(10000*EXP(3380*(1/(273.15+19)-1/(273.15+25))),0)
    1213,//3,//=ROUND(10000*EXP(3380*(1/(273.15+20)-1/(273.15+25))),0)
    1166,//7,//=ROUND(10000*EXP(3380*(1/(273.15+21)-1/(273.15+25))),0)
    1122,//1,//=ROUND(10000*EXP(3380*(1/(273.15+22)-1/(273.15+25))),0)
    1079,//6,//=ROUND(10000*EXP(3380*(1/(273.15+23)-1/(273.15+25))),0)
    1038,//9,//=ROUND(10000*EXP(3380*(1/(273.15+24)-1/(273.15+25))),0)
    1000,//0,//=ROUND(10000*EXP(3380*(1/(273.15+25)-1/(273.15+25))),0)
    962,//8,//=ROUND(10000*EXP(3380*(1/(273.15+26)-1/(273.15+25))),0)
    927,//2,//=ROUND(10000*EXP(3380*(1/(273.15+27)-1/(273.15+25))),0)
    893,//2,//=ROUND(10000*EXP(3380*(1/(273.15+28)-1/(273.15+25))),0)
    860,//6,//=ROUND(10000*EXP(3380*(1/(273.15+29)-1/(273.15+25))),0)
    829,//5,//=ROUND(10000*EXP(3380*(1/(273.15+30)-1/(273.15+25))),0)
    799,//6,//=ROUND(10000*EXP(3380*(1/(273.15+31)-1/(273.15+25))),0)
    771,//0,//=ROUND(10000*EXP(3380*(1/(273.15+32)-1/(273.15+25))),0)
    743,//6,//=ROUND(10000*EXP(3380*(1/(273.15+33)-1/(273.15+25))),0)
    717,//4,//=ROUND(10000*EXP(3380*(1/(273.15+34)-1/(273.15+25))),0)
    692,//2,//=ROUND(10000*EXP(3380*(1/(273.15+35)-1/(273.15+25))),0)
    668,//1,//=ROUND(10000*EXP(3380*(1/(273.15+36)-1/(273.15+25))),0)
    644,//9,//=ROUND(10000*EXP(3380*(1/(273.15+37)-1/(273.15+25))),0)
    622,//7,//=ROUND(10000*EXP(3380*(1/(273.15+38)-1/(273.15+25))),0)
    601,//4,//=ROUND(10000*EXP(3380*(1/(273.15+39)-1/(273.15+25))),0)
    581,//0,//=ROUND(10000*EXP(3380*(1/(273.15+40)-1/(273.15+25))),0)
    561,//4,//=ROUND(10000*EXP(3380*(1/(273.15+41)-1/(273.15+25))),0)
    542,//5,//=ROUND(10000*EXP(3380*(1/(273.15+42)-1/(273.15+25))),0)
    524,//4,//=ROUND(10000*EXP(3380*(1/(273.15+43)-1/(273.15+25))),0)
    507,//0,//=ROUND(10000*EXP(3380*(1/(273.15+44)-1/(273.15+25))),0)
    490,//3,//=ROUND(10000*EXP(3380*(1/(273.15+45)-1/(273.15+25))),0)
    474,//3,//=ROUND(10000*EXP(3380*(1/(273.15+46)-1/(273.15+25))),0)
    458,//9,//=ROUND(10000*EXP(3380*(1/(273.15+47)-1/(273.15+25))),0)
    444,//0,//=ROUND(10000*EXP(3380*(1/(273.15+48)-1/(273.15+25))),0)
    429,//7,//=ROUND(10000*EXP(3380*(1/(273.15+49)-1/(273.15+25))),0)
    416,//0,//=ROUND(10000*EXP(3380*(1/(273.15+50)-1/(273.15+25))),0)
    396,//9,//=ROUND(10000*EXP(3435*(1/(273.15+51)-1/(273.15+25))),0)
    384,//2,//=ROUND(10000*EXP(3435*(1/(273.15+52)-1/(273.15+25))),0)
    371,//9,//=ROUND(10000*EXP(3435*(1/(273.15+53)-1/(273.15+25))),0)
    360,//1,//=ROUND(10000*EXP(3435*(1/(273.15+54)-1/(273.15+25))),0)
    348,//8,//=ROUND(10000*EXP(3435*(1/(273.15+55)-1/(273.15+25))),0)
    337,//9,//=ROUND(10000*EXP(3435*(1/(273.15+56)-1/(273.15+25))),0)
    327,//4,//=ROUND(10000*EXP(3435*(1/(273.15+57)-1/(273.15+25))),0)
    317,//2,//=ROUND(10000*EXP(3435*(1/(273.15+58)-1/(273.15+25))),0)
    307,//5,//=ROUND(10000*EXP(3435*(1/(273.15+59)-1/(273.15+25))),0)
    298,//1,//=ROUND(10000*EXP(3435*(1/(273.15+60)-1/(273.15+25))),0)
    289,//0,//=ROUND(10000*EXP(3435*(1/(273.15+61)-1/(273.15+25))),0)
    280,//3,//=ROUND(10000*EXP(3435*(1/(273.15+62)-1/(273.15+25))),0)
    271,//9,//=ROUND(10000*EXP(3435*(1/(273.15+63)-1/(273.15+25))),0)
    263,//8,//=ROUND(10000*EXP(3435*(1/(273.15+64)-1/(273.15+25))),0)
    255,//9,//=ROUND(10000*EXP(3435*(1/(273.15+65)-1/(273.15+25))),0)
    248,//4,//=ROUND(10000*EXP(3435*(1/(273.15+66)-1/(273.15+25))),0)
    241,//1,//=ROUND(10000*EXP(3435*(1/(273.15+67)-1/(273.15+25))),0)
    234,//1,//=ROUND(10000*EXP(3435*(1/(273.15+68)-1/(273.15+25))),0)
    227,//3,//=ROUND(10000*EXP(3435*(1/(273.15+69)-1/(273.15+25))),0)
    220,//7,//=ROUND(10000*EXP(3435*(1/(273.15+70)-1/(273.15+25))),0)
    214,//4,//=ROUND(10000*EXP(3435*(1/(273.15+71)-1/(273.15+25))),0)
    208,//3,//=ROUND(10000*EXP(3435*(1/(273.15+72)-1/(273.15+25))),0)
    202,//4,//=ROUND(10000*EXP(3435*(1/(273.15+73)-1/(273.15+25))),0)
    196,//7,//=ROUND(10000*EXP(3435*(1/(273.15+74)-1/(273.15+25))),0)
    191,//2,//=ROUND(10000*EXP(3435*(1/(273.15+75)-1/(273.15+25))),0)
    185,//8,//=ROUND(10000*EXP(3435*(1/(273.15+76)-1/(273.15+25))),0)
    180,//7,//=ROUND(10000*EXP(3435*(1/(273.15+77)-1/(273.15+25))),0)
    175,//7,//=ROUND(10000*EXP(3435*(1/(273.15+78)-1/(273.15+25))),0)
    170,//9,//=ROUND(10000*EXP(3435*(1/(273.15+79)-1/(273.15+25))),0)
    166,//2,//=ROUND(10000*EXP(3435*(1/(273.15+80)-1/(273.15+25))),0)
};
#endif