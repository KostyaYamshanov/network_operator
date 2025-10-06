#include "ganop.hpp"

int main() {
    GANOP ga(4, 16, 16, 10, 500, 4); // p=1, c=8, d=24, lchr=10, HH=3, nfu=1
    ga.GenAlgorithm();
    return 0;
}