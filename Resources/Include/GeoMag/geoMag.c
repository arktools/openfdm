#ifndef GEOMAG_C
#define GEOMAG_C
void geoMag(double x1, double x2, double* y1, double* y2) {
    *y1 = x1+x2;
    *y2 = x1-x2;
}
#endif
