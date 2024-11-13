#include <iostream>
#include <vector>
#include <stdexcept>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

class KalmanFilter {
public:
    KalmanFilter(const std::vector<std::vector<double>>& A,
                 const std::vector<std::vector<double>>& B,
                 const std::vector<std::vector<double>>& H,
                 const std::vector<std::vector<double>>& Q,
                 const std::vector<std::vector<double>>& R,
                 const std::vector<double>& initial_state,
                 const std::vector<std::vector<double>>& initial_covariance)
        : A_(A), B_(B), H_(H), Q_(Q), R_(R), 
          state_(initial_state), covariance_(initial_covariance) {}

    void predict(const std::vector<double>& control) {
        state_ = mat_vec_mult(A_, state_);
        if (!B_.empty()) {
            auto control_effect = mat_vec_mult(B_, control);
            for (size_t i = 0; i < state_.size(); ++i) {
                state_[i] += control_effect[i];
            }
        }
        covariance_ = mat_add(mat_mult(A_, mat_mult(covariance_, transpose(A_))), Q_);
    }

    void update(const std::vector<double>& measurement) {
        auto y = vec_sub(measurement, mat_vec_mult(H_, state_));
        auto S = mat_add(mat_mult(H_, mat_mult(covariance_, transpose(H_))), R_);
        auto K = mat_mult(covariance_, mat_mult(transpose(H_), invert(S)));
        state_ = vec_add(state_, mat_vec_mult(K, y));
        covariance_ = mat_mult(mat_sub(identity(covariance_.size()), mat_mult(K, H_)), covariance_);
    }

    std::vector<double> get_state() const {
        return state_;
    }

private:
    std::vector<std::vector<double>> A_, B_, H_, Q_, R_, covariance_;
    std::vector<double> state_;

    std::vector<std::vector<double>> mat_mult(const std::vector<std::vector<double>>& m1, 
                                              const std::vector<std::vector<double>>& m2) {
        std::vector<std::vector<double>> result(m1.size(), std::vector<double>(m2[0].size(), 0.0));
        for (size_t i = 0; i < m1.size(); ++i)
            for (size_t j = 0; j < m2[0].size(); ++j)
                for (size_t k = 0; k < m2.size(); ++k)
                    result[i][j] += m1[i][k] * m2[k][j];
        return result;
    }

    std::vector<double> mat_vec_mult(const std::vector<std::vector<double>>& m, 
                                     const std::vector<double>& v) {
        std::vector<double> result(m.size(), 0.0);
        for (size_t i = 0; i < m.size(); ++i)
            for (size_t j = 0; j < v.size(); ++j)
                result[i] += m[i][j] * v[j];
        return result;
    }

    std::vector<double> vec_add(const std::vector<double>& v1, 
                                const std::vector<double>& v2) {
        std::vector<double> result(v1.size());
        for (size_t i = 0; i < v1.size(); ++i)
            result[i] = v1[i] + v2[i];
        return result;
    }

    std::vector<double> vec_sub(const std::vector<double>& v1, 
                                const std::vector<double>& v2) {
        std::vector<double> result(v1.size());
        for (size_t i = 0; i < v1.size(); ++i)
            result[i] = v1[i] - v2[i];
        return result;
    }

    std::vector<std::vector<double>> mat_add(const std::vector<std::vector<double>>& m1, 
                                             const std::vector<std::vector<double>>& m2) {
        std::vector<std::vector<double>> result(m1.size(), std::vector<double>(m1[0].size()));
        for (size_t i = 0; i < m1.size(); ++i)
            for (size_t j = 0; j < m1[0].size(); ++j)
                result[i][j] = m1[i][j] + m2[i][j];
        return result;
    }

    std::vector<std::vector<double>> mat_sub(const std::vector<std::vector<double>>& m1, 
                                             const std::vector<std::vector<double>>& m2) {
        std::vector<std::vector<double>> result(m1.size(), std::vector<double>(m1[0].size()));
        for (size_t i = 0; i < m1.size(); ++i)
            for (size_t j = 0; j < m1[0].size(); ++j)
                result[i][j] = m1[i][j] - m2[i][j];
        return result;
    }

    std::vector<std::vector<double>> transpose(const std::vector<std::vector<double>>& m) {
        std::vector<std::vector<double>> result(m[0].size(), std::vector<double>(m.size()));
        for (size_t i = 0; i < m.size(); ++i)
            for (size_t j = 0; j < m[0].size(); ++j)
                result[j][i] = m[i][j];
        return result;
    }

    std::vector<std::vector<double>> identity(size_t size) {
        std::vector<std::vector<double>> result(size, std::vector<double>(size, 0.0));
        for (size_t i = 0; i < size; ++i)
            result[i][i] = 1.0;
        return result;
    }

    std::vector<std::vector<double>> invert(const std::vector<std::vector<double>>& m) {
        size_t n = m.size();
        std::vector<std::vector<double>> augmented_matrix(n, std::vector<double>(2 * n));

        // Crear matriz aumentada [m | identidad]
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                augmented_matrix[i][j] = m[i][j];
            }
            augmented_matrix[i][n + i] = 1.0;
        }

        // Aplicar eliminación de Gauss-Jordan
        for (size_t i = 0; i < n; ++i) {
            double pivot = augmented_matrix[i][i];
            if (pivot == 0) throw std::runtime_error("Matrix is singular and cannot be inverted.");

            // Normalizar la fila del pivote
            for (size_t j = 0; j < 2 * n; ++j) {
                augmented_matrix[i][j] /= pivot;
            }

            // Hacer ceros en la columna del pivote en otras filas
            for (size_t k = 0; k < n; ++k) {
                if (k != i) {
                    double factor = augmented_matrix[k][i];
                    for (size_t j = 0; j < 2 * n; ++j) {
                        augmented_matrix[k][j] -= factor * augmented_matrix[i][j];
                    }
                }
            }
        }

        // Extraer la parte derecha de la matriz aumentada como la inversa
        std::vector<std::vector<double>> inverse(n, std::vector<double>(n));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                inverse[i][j] = augmented_matrix[i][j + n];
            }
        }
        return inverse;
    }
};

PYBIND11_MODULE(kalman_filter, m) {
    py::class_<KalmanFilter>(m, "KalmanFilter")
        .def(py::init<const std::vector<std::vector<double>>&,
                      const std::vector<std::vector<double>>&,
                      const std::vector<std::vector<double>>&,
                      const std::vector<std::vector<double>>&,
                      const std::vector<std::vector<double>>&,
                      const std::vector<double>&,
                      const std::vector<std::vector<double>>&>())
        .def("predict", &KalmanFilter::predict)
        .def("update", &KalmanFilter::update)
        .def("get_state", &KalmanFilter::get_state);
}
