/*
* Autor: Navil Abdeselam Abdel-lah
* Fecha: 25/11/2022
* Org: UAHR Krakens
*
* Librería para computación de las operaciones correspondientes
* al filtro de kalman utilizado para la obtención estimada de la
* pose del robot.
*
*/

#include "../include/ekf_krakens.hpp"

EKFilter::EKFilter( const Eigen::MatrixXd& A,
                    const Eigen::MatrixXd& B,
                    const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& Q,
                    const Eigen::MatrixXd& R,
                    const Eigen::MatrixXd& P,
                    const Eigen::VectorXd& x0,
                    const Eigen::VectorXd& u0)
                    :A(A), H(H), Q(Q), R(R), P(P), m(H.rows()), n(A.rows()), I(A.rows(),A.rows()), x_hat(x0), x_hat_new(x0), u(u0)
{
    // Necesitamos la matriz unidad para operar con ella.
    I.setIdentity();
}

EKFilter::EKFilter( const Eigen::MatrixXd& A,
                    const Eigen::MatrixXd& B,
                    const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& Q,
                    const Eigen::MatrixXd& R,
                    const Eigen::MatrixXd& P)
                    :A(A), H(H), Q(Q), R(R), P(P), m(H.rows()), n(A.rows()), I(A.rows(),A.rows()), x_hat(A.rows()), x_hat_new(A.rows()), u(B.rows())
{
    // Necesitamos la matriz unidad para operar con ella.
    I.setIdentity();
    // Si no se nos manda un valor inicial suponemos que es cero.
    x_hat.setZero();
    x_hat_new.setZero();
    // Con la señal de control ocurre lo mismo, suponemos que es cero.
    u.setZero();
}

EKFilter::EKFilter( const Eigen::MatrixXd& A,
                    const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& Q,
                    const Eigen::MatrixXd& R,
                    const Eigen::MatrixXd& P)
                    :A(A), H(H), Q(Q), R(R), P(P), m(H.rows()), n(A.rows()), I(A.rows(),A.rows()), x_hat(A.rows()), x_hat_new(A.rows()), B(A.rows(),A.rows()), u(A.rows())
{
    // Necesitamos la matriz unidad para operar con ella.
    I.setIdentity();
    // Si no se nos manda un valor inicial suponemos que es cero.
    x_hat.setZero();
    x_hat_new.setZero();
    // Damos valores nulos a B y u para que no molesten en runtime.
    B.setZero();
    u.setZero();
}

EKFilter::EKFilter( const Eigen::MatrixXd& A,
                    const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& Q,
                    const Eigen::MatrixXd& R,
                    const Eigen::MatrixXd& P,
                    const Eigen::VectorXd& x0)
                    :A(A), H(H), Q(Q), R(R), P(P), m(H.rows()), n(A.rows()), I(A.rows(),A.rows()), x_hat(x0), x_hat_new(x0), B(A.rows(),A.rows()), u(A.rows())
{
    // Necesitamos la matriz unidad para operar con ella.
    I.setIdentity();
    // Damos valores nulos a B y u para que no molesten en runtime.
    B.setZero();
    u.setZero();
}

Eigen::VectorXd& EKFilter::prediction(const Eigen::VectorXd& u_new)
{
    // Actualizamos la señal de control.
    u = u_new;

    // Actualización de predicción.
    x_hat_new   = A*x_hat + B*u;
    P           = A*P*A.transpose() + Q;

    return x_hat_new;
}

Eigen::VectorXd& EKFilter::prediction()
{
    // Actualización de predicción sin señal de control alguna.
    x_hat_new   = A*x_hat;
    P           = A*P*A.transpose() + Q;

    return x_hat_new;
}

Eigen::VectorXd& EKFilter::update(const Eigen::VectorXd& z)
{
    // Actualización de las medidas.
    K           = P*H.transpose()*(H*P.inverse()*H.transpose() + R).inverse();
    x_hat_new   += K*(z - H*x_hat_new);
    P           = (I - K*H)*P;

    // Último valor del vector de estados...
    x_hat = x_hat_new;

    return x_hat;
}

Eigen::VectorXd& EKFilter::update()
{
    // Actualización de las medidas.
    K           = P*H.transpose()*(H*P.inverse()*H.transpose() + R).inverse();
    x_hat_new   += K*(H*x_hat - H*x_hat_new);
    P           = (I - K*H)*P;

    // Último valor del vector de estados...
    x_hat = x_hat_new;

    return x_hat;
}