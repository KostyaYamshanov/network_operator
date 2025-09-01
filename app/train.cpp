#include "ganop.hpp"

int main() {
    GANOP ga(3, 8, 24, 32, 1024, 2); // p=1, c=8, d=24, lchr=10, HH=3, nfu=1
    ga.GenAlgorithm();
    return 0;
}