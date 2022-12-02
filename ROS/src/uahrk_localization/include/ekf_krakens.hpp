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

#pragma once

#include <Eigen/Dense>

class EKFilter
{
    public:

        // Constructor con valor inicial. 
        EKFilter(   
                    const Eigen::MatrixXd& A,
                    const Eigen::MatrixXd& B,
                    const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& Q,
                    const Eigen::MatrixXd& R,
                    const Eigen::MatrixXd& P,
                    const Eigen::VectorXd& x0,
                    const Eigen::VectorXd& u0
                    );
        
        // Constructor sin valor inicial -> Suponemos valor inicial nulo.
        EKFilter(
                    const Eigen::MatrixXd& A,
                    const Eigen::MatrixXd& B,
                    const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& Q,
                    const Eigen::MatrixXd& R,
                    const Eigen::MatrixXd& P
                );
        
        // Constructor sin señal de control -> Suponemos que no hay señal de control.
        EKFilter(
                    const Eigen::MatrixXd& A,
                    const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& Q,
                    const Eigen::MatrixXd& R,
                    const Eigen::MatrixXd& P
                );

        EKFilter( const Eigen::MatrixXd& A,
                    const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& Q,
                    const Eigen::MatrixXd& R,
                    const Eigen::MatrixXd& P,
                    const Eigen::VectorXd& x0);

        // Predicción del filtro de Kalman 
        Eigen::VectorXd& prediction(const Eigen::VectorXd& u_new);
        // Predicción del filtro de Kalman sin señal de control
        Eigen::VectorXd& prediction();

        // Estimación de la pose. -> z es el vector de medidas.
        Eigen::VectorXd& update(const Eigen::VectorXd& z); 
        // Si no le pasamos medidas refresca con el último valor emitido.
        Eigen::VectorXd& update();

        // Estados de la dinámica del filtro.
        Eigen::VectorXd& state(){return x_hat;}

    private:

        // Matrices.
        Eigen::MatrixXd A, B, H, Q, R, P, K, P0, I; 

        // Dimensiones del sistema.
        int m, n;

        // Estado estimado en k y k-1
        Eigen::VectorXd x_hat, x_hat_new;

        // Señal de control para la predicción
        Eigen::VectorXd u;

};