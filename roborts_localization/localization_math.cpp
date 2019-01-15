/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "localization_math.h"

namespace roborts_localization {
namespace math {

Vec3d CoordAdd(const Vec3d &a, const Vec3d &b) {
  Vec3d c;
  c(0) = b(0) + a(0) * std::cos(b(2)) - a(1) * std::sin(b(2));
  c(1) = b(1) + a(0) * std::sin(b(2)) + a(1) * std::cos(b(2));
  c(2) = b(2) + a(2);
  c(2) = std::atan2(sin(c(2)), cos(c(2)));
  return c;
};

void EigenDecomposition(const Mat3d &matrix_a,
                        Mat3d &matrix_v,
                        Vec3d &vector_d) {
  int i, j;
  Vec3d vector_e;
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      (matrix_v)(i, j) = matrix_a(i, j);
    }
  }

  Tred2(matrix_v, vector_d, vector_e);
  Tql2(matrix_v, vector_d, vector_e);

}

inline void Tred2(Mat3d &matrix_v, Vec3d &vector_d, Vec3d &vector_e) {

//  This is derived from the Algol procedures tred2 by
//  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
//  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
//  Fortran subroutine in EISPACK.

  int i, j, k;
  double f, g, h, hh;
  for (j = 0; j < 3; j++) {
    vector_d(j) = matrix_v(2, j);
  }

  // Householder reduction to tridiagonal form.

  for (i = 3 - 1; i > 0; i--) {

    // Scale to avoid under/overflow.

    double scale = 0.0;
    double h = 0.0;
    for (k = 0; k < i; k++) {
      scale = scale + fabs(vector_d(k));
    }
    if (scale == 0.0) {
      vector_e[i] = vector_d(i - 1);
      for (j = 0; j < i; j++) {
        vector_d(j) = matrix_v(i - 1, j);
        matrix_v(i, j) = 0.0;
        matrix_v(j, i) = 0.0;
      }
    } else {

      // Generate Householder vector.

      for (k = 0; k < i; k++) {
        vector_d(k) /= scale;
        h += vector_d(k) * vector_d(k);
      }
      f = vector_d(i - 1);
      g = sqrt(h);
      if (f > 0) {
        g = -g;
      }
      vector_e(i) = scale * g;
      h = h - f * g;
      vector_d(i - 1) = f - g;
      for (j = 0; j < i; j++) {
        vector_e(j) = 0.0;
      }

      // Apply similarity transformation to remaining columns.

      for (j = 0; j < i; j++) {
        f = vector_d(j);
        matrix_v(j, i) = f;
        g = vector_e(j) + matrix_v(j, j) * f;
        for (k = j + 1; k <= i - 1; k++) {
          g += matrix_v(k, j) * vector_d(k);
          vector_e[k] += matrix_v(k, j) * f;
        }
        vector_e[j] = g;
      }
      f = 0.0;
      for (j = 0; j < i; j++) {
        vector_e(j) /= h;
        f += vector_e(j) * vector_d(j);
      }
      hh = f / (h + h);
      for (j = 0; j < i; j++) {
        vector_e(j) -= hh * vector_d(j);
      }
      for (j = 0; j < i; j++) {
        f = vector_d(j);
        g = vector_e(j);
        for (k = j; k <= i - 1; k++) {
          matrix_v(k, j) -= (f * vector_e(k) + g * vector_d(k));
        }
        vector_d[j] = matrix_v(i - 1, j);
        matrix_v(i, j) = 0.0;
      }
    }
    vector_d(i) = h;
  }

  // Accumulate transformations.

  for (i = 0; i < 2; i++) {
    matrix_v(2, i) = matrix_v(i, i);
    matrix_v(i, i) = 1.0;
    h = vector_d(i + 1);
    if (h != 0.0) {
      for (k = 0; k <= i; k++) {
        vector_d(k) = matrix_v(k, i + 1) / h;
      }
      for (j = 0; j <= i; j++) {
        g = 0.0;
        for (k = 0; k <= i; k++) {
          g += matrix_v(k, i + 1) * matrix_v(k, j);
        }
        for (k = 0; k <= i; k++) {
          matrix_v(k, j) -= g * vector_d(k);
        }
      }
    }
    for (k = 0; k <= i; k++) {
      matrix_v(k, i + 1) = 0.0;
    }
  }
  for (j = 0; j < 3; j++) {
    vector_d(j) = matrix_v(2, j);
    matrix_v(2, j) = 0.0;
  }
  matrix_v(2, 2) = 1.0;
  vector_e(0) = 0.0;
}

inline void Tql2(Mat3d &matrix_v, Vec3d &vector_d, Vec3d &vector_e) {

//  This is derived from the Algol procedures tql2, by
//  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
//  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
//  Fortran subroutine in EISPACK.

  int i, j, m, l, k;
  double g, p, r, dl1, h, f, tst1, eps;
  double c, c2, c3, el1, s, s2;

  for (i = 1; i < 3; i++) {
    vector_e[i - 1] = vector_e[i];
  }
  vector_e[3 - 1] = 0.0;

  f = 0.0;
  tst1 = 0.0;
  eps = pow(2.0, -52.0);
  for (l = 0; l < 3; l++) {

    // Find small subdiagonal element

    tst1 = std::max(tst1, fabs(vector_d[l]) + fabs(vector_e[l]));
    m = l;
    while (m < 3) {
      if (fabs(vector_e[m]) <= eps * tst1) {
        break;
      }
      m++;
    }

    // If m == l, d[l] is an eigenvalue,
    // otherwise, iterate.

    if (m > l) {
      int iter = 0;
      do {
        iter = iter + 1;  // (Could check iteration count here.)

        // Compute implicit shift

        g = vector_d[l];
        p = (vector_d[l + 1] - g) / (2.0 * vector_e[l]);
        r = std::sqrt(p * p + 1.0 * 1.0);
//        r = hypot2(p,1.0);
        if (p < 0) {
          r = -r;
        }
        vector_d[l] = vector_e[l] / (p + r);
        vector_d[l + 1] = vector_e[l] * (p + r);
        dl1 = vector_d[l + 1];
        h = g - vector_d[l];
        for (i = l + 2; i < 3; i++) {
          vector_d[i] -= h;
        }
        f = f + h;

        // Implicit QL transformation.

        p = vector_d[m];
        c = 1.0;
        c2 = c;
        c3 = c;
        el1 = vector_e[l + 1];
        s = 0.0;
        s2 = 0.0;
        for (i = m - 1; i >= l; i--) {
          c3 = c2;
          c2 = c;
          s2 = s;
          g = c * vector_e[i];
          h = c * p;
          r = std::sqrt(p * p + vector_e[i] * vector_e[i]);
          vector_e[i + 1] = s * r;
          s = vector_e[i] / r;
          c = p / r;
          p = c * vector_d[i] - s * g;
          vector_d[i + 1] = h + s * (c * g + s * vector_d[i]);

          // Accumulate transformation.

          for (k = 0; k < 3; k++) {
            h = matrix_v(k, i + 1);
            matrix_v(k, i + 1) = s * matrix_v(k, i) + c * h;
            matrix_v(k, i) = c * matrix_v(k, i) - s * h;
          }
        }
        p = -s * s2 * c3 * el1 * vector_e[l] / dl1;
        vector_e[l] = s * p;
        vector_d[l] = c * p;

        // Check for convergence.

      } while (fabs(vector_e[l]) > eps * tst1);
    }
    vector_d[l] = vector_d[l] + f;
    vector_e[l] = 0.0;
  }

  // Sort eigenvalues and corresponding vectors.

  for (i = 0; i < 3 - 1; i++) {
    k = i;
    p = vector_d[i];
    for (j = i + 1; j < 3; j++) {
      if (vector_d[j] < p) {
        k = j;
        p = vector_d[j];
      }
    }
    if (k != i) {
      vector_d[k] = vector_d[i];
      vector_d[i] = p;
      for (j = 0; j < 3; j++) {
        p = matrix_v(j, i);
        matrix_v(j, i) = matrix_v(j, k);
        matrix_v(j, k) = p;
      }
    }
  }
}

}// math

}// roborts_localization
