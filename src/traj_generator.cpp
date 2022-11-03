#include <traj_generator.h>


/*
d888888b d888888b .88b  d88. d88888b .d8888.  .o88b.  .d8b.  db      d88888b 
`~~88~~'   `88'   88'YbdP`88 88'     88'  YP d8P  Y8 d8' `8b 88      88'     
   88       88    88  88  88 88ooooo `8bo.   8P      88ooo88 88      88ooooo 
   88       88    88  88  88 88~~~~~   `Y8b. 8b      88~~~88 88      88~~~~~ 
   88      .88.   88  88  88 88.     db   8D Y8b  d8 88   88 88booo. 88.     
   YP    Y888888P YP  YP  YP Y88888P `8888Y'  `Y88P' YP   YP Y88888P Y88888P 
*/

Vector3d TimeScale::first_order_poly(const double &s)
{
    Vector3d res = Vector3d::Zero();
    if (check(s))
{
        double s_new = s;
        double ds_new = 1.;
        double dds_new = 0.;
        res(0) = s_new;
        res(1) = ds_new;
        res(2) = dds_new;
    }
    return res;
}

/*
Description: cubic polynomial varying time scale
Input: T -> time period
        s -> t/T, linearly varying time scale [0,1]
Output: res -> the pointer of the return variable, should be size(3), contain {s_new, ds_new, dds_new}
        s_new -> cubic polynomial varying time scale [0,1]
        ds_new -> derivative of s_new
        dds_new -> derivative of ds_new       
*/
Vector3d TimeScale::third_order_poly(const double &T, const double &s)
{
    Vector3d res = Vector3d::Zero();
    if (check(s))
    {
        double s_new = 3*pow(s,2) - 2*pow(s,3);
        double ds_new = 6/T * s * (1-s);
        double dds_new = 6/pow(T,2) * (1 - 2*s);
        res(0) = s_new;
        res(1) = ds_new;
        res(2) = dds_new;
    }
    return res;
}

/*
Description: fifth order polynomial varying time scale      
*/
Vector3d TimeScale::fifth_order_poly(const double &T, const double &s)
{
    Vector3d res = Vector3d::Zero();
    if (check(s))
    {
        double s_new = 10*pow(s,3) - 15*pow(s,4) + 6*pow(s,5);
        double ds_new = 30/T * pow(s,2) * pow(s-1, 2);
        double dds_new = 60/pow(T,2) * s*(1 - 3*s + 2*pow(s,2));
        res(0) = s_new;
        res(1) = ds_new;
        res(2) = dds_new;
    }
    return res;
}

// under check
Vector3d TimeScale::quick_foot_fall(const double &T, const double &s)
{
    // Description: the first 50% time finish the first 80% path
    //              and the last 50% time finish the last 20% path  
    // two fifth-order polynomial s_new = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5 (s = t/T)
    // S1(0) = 0, dS1(0) = 0, ddS1(0) = 0
    // S1(T/2) = 0.8, dS1(T/2) = 1, ddS1(T/2) = 0
    // S2(T/2) = 0.8, dS2(T/2) = 1, ddS2(T/2) = 0
    // S2(T) = 1, dS2(T) = 0, ddS2(T) = 0
    // TODO: optimize the parameters that minimize (the max accleration or velocity)
    Vector3d res = Vector3d::Zero();
    if (check(s))
    {
        double s_new, ds_new, dds_new;
        if (s <= 0.5)
        {
            s_new = (64.0 - 16.0*T)*pow(s,3) - (192.0 - 56.0*T)*pow(s,4) + (153.6 - 48*T)*pow(s,5);
            ds_new = (3*(64.0 - 16.0*T)*pow(s,2) - 4*(192.0 - 56.0*T)*pow(s,3) + 5*(153.6 - 48*T)*pow(s,4))/T;
            dds_new = (6*(64.0 - 16.0*T)*s - 12*(192.0 - 56.0*T)*pow(s,2) + 20*(153.6 - 48*T)*pow(s,3))/T/T;
            
            // s_new = 1.6*s;
            // ds_new = 1.6;
            // dds_new = 0.;
        }
        else
        {
            s_new = -(5.4 - 8.0*T) + (48.0 - 64.0*T)*s - (144.0 - 192.0*T)*pow(s,2) + 
                    (208.0 - 272.0*T)*pow(s,3) - (144.0 - 184.0*T)*pow(s,4) + (38.4 - 48*T)*pow(s,5);
            ds_new = ((48.0 - 64.0*T) - 2*(144.0 - 192.0*T)*s + 3*(208.0 - 272.0*T)*pow(s,2) 
                    - 4*(144.0 - 184.0*T)*pow(s,3) + 5*(38.4 - 48*T)*pow(s,4))/T;
            dds_new = (- 2*(144.0 - 192.0*T) + 6*(208.0 - 272.0*T)*s 
                    - 12*(144.0 - 184.0*T)*pow(s,2) + 20*(38.4 - 48*T)*pow(s,3))/T/T;

            // s_new = 0.8 + 0.4*(s-0.5);
            // ds_new = 0.4;
            // dds_new = 0;
        }
        res(0) = s_new;
        res(1) = ds_new;
        res(2) = dds_new;
    }   
    return res;
}


/*
d8888b. .d888b. d8888b. 
88  `8D VP  `8D 88  `8D 
88oodD'    odD' 88oodD' 
88~~~    .88'   88~~~   
88      j88.    88      
88      888888D 88 
*/

/*
Description: point to point trajectory generation 3D
Input: q_start -> start point
        q_end -> end point
        T -> time period
        s -> t/T, linearly varying time scale [0,1]
        order -> time scale polynomial order, 1 or 3 or 5
Output: res -> the pointer of the return variable, should be size(3), contain {q, dq, ddq}      
*/
vector<Vector3d> Point2PointTrajGenerator::linear_traj(const Vector3d &q_start, 
                                                    const Vector3d &q_end, 
                                                    const double &T, 
                                                    const double &s, 
                                                    int order)
{
    vector<Vector3d> res(3,Vector3d::Zero());
    Vector3d res_s;
    if (order == 1){
        res_s = _time_scale.first_order_poly(s);
    }
    else if (order == 3){
        res_s = _time_scale.third_order_poly(T, s);
    }
    else if (order == 5){
        res_s = _time_scale.fifth_order_poly(T, s);
    }
    else {
        cout << "[Traj Error]: order should be 1/3/5" << endl;
        return res;
    } 

    res.at(0) = q_start + res_s[0]*(q_end - q_start);
    res.at(1) = res_s[1]*(q_end - q_start);
    res.at(2) = res_s[2]*(q_end - q_start);
    return res;
}

/*
Description: linear decoupled trajectory generation of two configuration
Input: T_start -> start configuration
        T_end -> end configuration
        T -> time period
        s -> t/T, linearly varying time scale [0,1] 
        order -> time scale polynomial order, 1 or 3 or 5
Output: T_res -> the pointer of the current configuration at s  
*/
pin::SE3 Point2PointTrajGenerator::linear_decoupled_traj(const pin::SE3 &T_start, 
                                                    const pin::SE3 &T_end, 
                                                    const double &T,
                                                    const double &s,  
                                                    int order)
{
    pin::SE3 T_res(Matrix3d::Zero(),Vector3d::Zero());
    Vector3d p_start, p_end, p;
    Matrix3d R_start, R_end, R;
    Vector3d res_s;
    if (order == 1){
        res_s = _time_scale.first_order_poly(s);
    }
    else if (order == 3){
        res_s = _time_scale.third_order_poly(T, s);
    }
    else if (order == 5){
        res_s = _time_scale.fifth_order_poly(T, s);
    }
    else {
        cout << "[Traj Error]: order should be 1/3/5" << endl;
        return T_res;
    }

    p_start = T_start.translation();
    R_start = T_start.rotation();
    p_end = T_end.translation();
    R_end = T_end.rotation();
    p = p_start + res_s[0] * (p_end - p_start);
    R = R_start * pin::exp3(res_s[0] * pin::log3(R_start.transpose() * R_end));
    T_res = pin::SE3(R, p);
    return T_res;
}

/*
Description: linear coupled trajectory generation of two configuration    
*/
pin::SE3 Point2PointTrajGenerator::linear_coupled_traj(const pin::SE3 &T_start, 
                                                    const pin::SE3 &T_end, 
                                                    const double &T,
                                                    const double &s, 
                                                    int order)
{
    pin::SE3 T_res(Matrix3d::Zero(),Vector3d::Zero());
    Vector3d res_s;
    if (order == 1){
        res_s = _time_scale.first_order_poly(s);
    }
    else if (order == 3){
        res_s = _time_scale.third_order_poly(T, s);
    }
    else if (order == 5){
        res_s = _time_scale.fifth_order_poly(T, s);
    }
    else {
        cout << "[Traj Error]: order should be 1/3/5" << endl;
        return T_res;
    }

    T_res = T_start.act(pin::exp6(res_s[0] * pin::log6(T_start.actInv(T_end))));
    return T_res;
}


/*
 .o88b. db    db  .o88b. db       .d88b.  d888888b d8888b. 
d8P  Y8 `8b  d8' d8P  Y8 88      .8P  Y8.   `88'   88  `8D 
8P       `8bd8'  8P      88      88    88    88    88   88 
8b         88    8b      88      88    88    88    88   88 
Y8b  d8    88    Y8b  d8 88booo. `8b  d8'   .88.   88  .8D 
 `Y88P'    YP     `Y88P' Y88888P  `Y88P'  Y888888P Y8888D' 
*/

/*
Description: cycloid trajectory generation
Input: q_start -> start point
        q_end -> end point
        T -> time period
        s -> t/T, linearly varying time scale [0,1]
        res -> the pointer of the return variable, should be size(3)
Output: res -> contain {q, dq, ddq}      
*/
vector<Vector3d> CycloidTrajGenerator::sw_traj(const Vector3d & p_hf_start, 
                                                const Vector3d & p_hf_end, 
                                                const double & h_gc, 
                                                const double &T_sw, 
                                                const double &s_sw)
{
    vector<Vector3d> res(3,Vector3d::Zero());
    if (_time_scale.check(s_sw))
    {
        double sgn;
        Vector3d temp1, temp2, temp3;
        
        if (s_sw >= 0. && s_sw < 0.5) {sgn = 1.;}
        else {sgn = -1.;}
        Vector3d step_length = p_hf_end - p_hf_start;    
        // position
        temp1 << 0., 0., h_gc * (sgn * (2*s_sw - 0.5/M_PI*sin(4.*M_PI*s_sw)-1.) + 1.);
        res.at(0) = p_hf_start + step_length * (s_sw - 0.5/M_PI*sin(2.*M_PI*s_sw)) + temp1;
        // velocity
        temp2 << 0., 0., h_gc * 2. / T_sw * sgn * (1. - cos(4*M_PI*s_sw));
        res.at(1) = step_length / T_sw * (1 - cos(2.*M_PI*s_sw)) + temp2;
        // accleration
        temp3 << 0., 0., h_gc * 8.*M_PI / T_sw / T_sw * sgn * sin(4*M_PI*s_sw);
        res.at(2) = step_length * 2.*M_PI / T_sw / T_sw * sin(2.*M_PI*s_sw) + temp3;
    }
    return res;
}

/*
Description: cycloid trajectory generation
Note: only for translation witout orientation
*/
vector<Vector3d> CycloidTrajGenerator::st_traj(const Vector3d & p_hf_start, 
                                            const Vector3d & p_hf_end, 
                                            const double &T_st, 
                                            const double &s_st)
{
    vector<Vector3d> res(3,Vector3d::Zero());
    if (_time_scale.check(s_st))
    {
        Vector3d step_length = p_hf_end - p_hf_start;
        // position
        res.at(0) = p_hf_end - step_length * (s_st - 0.5/M_PI*sin(2.*M_PI*s_st));
        // velocity
        res.at(1) = -step_length / T_st * (1 - cos(2.*M_PI*s_st));
        // accleration
        res.at(2) = -step_length * 2.*M_PI / T_st / T_st * sin(2.*M_PI*s_st);
    }
    return res;
}


/*
d8888b. d88888b .d8888. d888888b d88888D d88888b d8888b. 
88  `8D 88'     88'  YP   `88'   YP  d8' 88'     88  `8D 
88oooY' 88ooooo `8bo.      88       d8'  88ooooo 88oobY' 
88~~~b. 88~~~~~   `Y8b.    88      d8'   88~~~~~ 88`8b   
88   8D 88.     db   8D   .88.    d8' db 88.     88 `88. 
Y8888P' Y88888P `8888Y' Y888888P d88888P Y88888P 88   YD
*/

/*
Description: the bernstein polynomial of n,k as a function of s
Input: s -> time scale [0,1]
Output: bernstein polynomial
Note: https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html
*/
double BezierTrajGenerator::_bernstein_ploy(const int &k, const int &n, const double &s)
{
    double comb, B;
    comb = _factorial(n) / (_factorial(k) * _factorial(n-k));
    B = comb * pow(s,k) * pow(1-s, n-k);
    return B;
}

/*
Description: the bezier curve as a function of control_points
Input: control points should be a MatrixX3d:
        [[x1,y1,z1],
        [x2,y2,z2], 
        ..
        [Xn, Yn, Zn]] -> (n+1,3)
        T -> time period
        s -> t/T, linearly varying time scale [0,1]
        res -> the pointer of the return variable, should be size(3)
Output: res -> contain {p_d, v_d, a_d}      
*/
vector<Vector3d> BezierTrajGenerator::_bezier_traj(const MatrixX3d &ctrl_points, 
                                                    const double &T, 
                                                    const double &s)
{
    int n = ctrl_points.rows() - 1;
    RowVectorXd B_vec(n+1);
    RowVectorXd dB_vec(n);
    RowVectorXd ddB_vec(n-1);
    MatrixX3d dctrl_points, ddctrl_points;
    Vector3d p_d, v_d, a_d;
    // position
    int k = 0;
    for (k = 0; k < n+1; k++){
        B_vec(k) = _bernstein_ploy(k, n, s); // 1x(n+1)
    }
    p_d = (B_vec * ctrl_points).transpose(); // (1x(n+1) * (n+1)x3)^T = 3x1 
    // velocity
    for (k = 0; k < n; k++){
        dB_vec(k) = _bernstein_ploy(k, n-1, s); // 1xn
    }
    dctrl_points = (n * (ctrl_points.bottomRows(n).array() - ctrl_points.topRows(n).array())).matrix(); // nx3
    v_d = 1/T * ((dB_vec * dctrl_points).transpose()); // (1xn * nx3)^T = 3x1
    // acceleration
    for (k = 0; k < n-1; k++){
        ddB_vec(k) = _bernstein_ploy(k, n-2, s); // 1x(n-1)
    }
    ddctrl_points = ((n-1) * (dctrl_points.bottomRows(n-1).array() - dctrl_points.topRows(n-1).array())).matrix(); // (n-1)x3
    a_d = 1/T/T * ((ddB_vec * ddctrl_points).transpose()); // (1x(n-1) * (n-1)x3)^T = 3x1
    vector<Vector3d> res{p_d, v_d, a_d};
    return res;
}

/*
Description: The bezier trajectory for swing phase w.r.t hip frame
Input:  p_hf_start -> start point of foot w.r.t hip frame
        p_hf_end -> end point of foot w.r.t hip frame
        h_gc -> ground clearance w.r.t foot frame
        T_sw -> trajectory period
        s_sw -> time scale [0 ~ 1]
        res -> the pointer of the return variable, should be size(3)
Output: res -> contain {p_d, v_d, a_d}
Note: control points should be a MatrixX3d:
        [[x1,y1,z1],
        [x2,y2,z2], 
        ..
        [Xn, Yn, Zn]]
*/
vector<Vector3d> BezierTrajGenerator::sw_traj(const Vector3d &p_hf_start, 
                                            const Vector3d &p_hf_end, 
                                            const double &h_gc, 
                                            const double &T_sw, 
                                            const double &s_sw)
{    
    vector<Vector3d> res(3,Vector3d::Zero());
    if (_time_scale.check(s_sw))
    {
        // Vector3d step_length = p_hf_end - p_hf_start;
        // VectorXd xyz_scaler(14), h_scaler(14);
        // MatrixXd scaler = MatrixXd::Zero(14,3);
        // MatrixXd h = MatrixXd::Zero(14,3);
        // MatrixXd temp1 = MatrixXd::Zero(14,3);
        // MatrixXd temp2 = MatrixXd::Zero(14,3);
        // Matrix<double,14,3> ctrl_points;
        // xyz_scaler << 0., 0., 0., -0.1, -0.2, -0.2, 0.5, 0.5, 1.2, 1.2, 1.1, 1., 1., 1.; // (14,1)
        // scaler.colwise() += xyz_scaler; // 广播机制，每一列都加上xyz_scaler (14,3)
        // h_scaler << 0., 0., 0., 0., 0.9, 0.9, 0.9, 1.2, 1.2, 1.2, 0., 0., 0., 0.; // (14,1)
        // h.col(2) = h_gc * h_scaler; // (14,3)
        // temp1.rowwise() += step_length.transpose(); // 每一行都加上step_length^T (14,3)
        // temp2.rowwise() += p_hf_start.transpose(); // 每一行都加上p_hf_start^T (14,3)
        // ctrl_points = (scaler.array() * temp1.array() + temp2.array() + h.array()).matrix(); // (14,3) * (14,3) + (14,3) + (14,3) = (14,3)

        // like cycloid
        Vector3d step_length = p_hf_end - p_hf_start;
        VectorXd xyz_scaler(8), h_scaler(8);
        MatrixXd scaler = MatrixXd::Zero(8,3);
        MatrixXd h = MatrixXd::Zero(8,3);
        MatrixXd temp1 = MatrixXd::Zero(8,3);
        MatrixXd temp2 = MatrixXd::Zero(8,3);
        Matrix<double,8,3> ctrl_points;
        xyz_scaler << 0., 0., 0., 0., 1., 1., 1., 1.; // (8,1)
        scaler.colwise() += xyz_scaler; // 广播机制，每一列都加上xyz_scaler (8,3)
        h_scaler << 0., 0., 0., 1.8, 1.8, 0., 0., 0.; // (8,1)
        h.col(2) = h_gc * h_scaler; // (8,3)
        temp1.rowwise() += step_length.transpose(); // 每一行都加上step_length^T (8,3)
        temp2.rowwise() += p_hf_start.transpose(); // 每一行都加上p_hf_start^T (8,3)
        ctrl_points = (scaler.array() * temp1.array() + temp2.array() + h.array()).matrix(); // (8,3) * (8,3) + (8,3) + (8,3) = (8,3)

        res = _bezier_traj(ctrl_points, T_sw, s_sw);
    }
    return res;
}

// /*
// Note: only for translation witout orientation
// */
// vector<Vector3d> BezierTrajGenerator::st_traj(const Vector3d &p_hf_start, 
//                                             const Vector3d &p_hf_end, 
//                                             const double &T_st, 
//                                             const double &s_st)
// {
//     vector<Vector3d> res(3,Vector3d::Zero());
//     if (_time_scale.check(s_st))
//     {
//         Vector3d step_length = p_hf_end - p_hf_start;
//         VectorXd xyz_scaler(6);
//         MatrixXd scaler = MatrixXd::Zero(6,3);
//         MatrixXd temp1 = MatrixXd::Zero(6,3);
//         MatrixXd temp2 = MatrixXd::Zero(6,3);
//         Matrix<double,6,3> ctrl_points;
//         xyz_scaler << 0., 0., 0., 1., 1., 1.; // (6,1)
//         scaler.colwise() += xyz_scaler; // 广播机制，每一列都加上xyz_scaler (6,3)
//         temp1.rowwise() += -step_length.transpose(); // 每一行都加上step_length^T (6,3)
//         temp2.rowwise() += p_hf_end.transpose(); // 每一行都加上p_hf_start^T (6,3)
//         ctrl_points = (scaler.array() * temp1.array() + temp2.array()).matrix(); // (6,3) * (6,3) + (6,3) + (6,3) = (6,3)

//         res = _bezier_traj(ctrl_points, T_st, s_st);
//     }
//     return res;
// }
