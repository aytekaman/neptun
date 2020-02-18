#include <floattetwild/FastWindingNumber.hpp>

#include <igl/parallel_for.h>

#include <windingnumber/UT_SolidAngle.h>

#include <cstdlib>



namespace floatTetWild {

void fast_winding_number(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, const Eigen::MatrixXd &P, Eigen::VectorXd &W)
{
    Eigen::Matrix<int,Eigen::Dynamic,3,Eigen::RowMajor> F_copy = F;

    HDK_Sample::UT_SolidAngle<float,float> solid_angle;

    int order = 2;
    double accuracy_scale = 2.0;

    std::vector<HDK_Sample::UT_Vector3T<float> > U(V.rows());
    for(int i = 0;i<V.rows();i++) {
        for(int j = 0;j<3;j++){
            U[i][j] = V(i,j);
        }
    }

    solid_angle.init(F_copy.rows(), F_copy.data(), V.rows(), &U[0], order);

    W.resize(P.rows());

    //for(int p = 0;p<P.rows();p++)
    igl::parallel_for(P.rows(),[&](int p) {
        HDK_Sample::UT_Vector3T<float>Pp;
        Pp[0] = P(p,0);
        Pp[1] = P(p,1);
        Pp[2] = P(p,2);
        W(p) = solid_angle.computeSolidAngle(Pp, accuracy_scale)/ (4.0*M_PI);
    } ,1000);
}
}