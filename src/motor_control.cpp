#include <motor_control.h>

/*
.88b  d88.  .d88b.  d888888b  .d88b.  d8888b. 
88'YbdP`88 .8P  Y8. `~~88~~' .8P  Y8. 88  `8D 
88  88  88 88    88    88    88    88 88oobY' 
88  88  88 88    88    88    88    88 88`8b   
88  88  88 `8b  d8'    88    `8b  d8' 88 `88. 
YP  YP  YP  `Y88P'     YP     `Y88P'  88   YD 
*/
MotorControl::MotorControl(SerialPort* serial_ptr, unsigned short id, double offset)
{
    this->_serial_ptr = serial_ptr;
    this->_offset = offset;
    this->_motor_s.id = id;
    this->_motor_s.motorType = MotorType::A1;
    this->_motor_r.motorType = this->_motor_s.motorType;
    // init motor states
    this->stop();
}

bool MotorControl::_cmd_is_safe(double tau_d, double dq_d, double q_d, double kp, double kd)
{
    if (fabs(tau_d) > MOTOR_TAU_MAX*SAFETY_FACTOR){ // 33.5Nm * 0.8 = 26.8
        cout << "[Motor " << _motor_s.id << "]: tau_d safety error\n";
        return false;
    } 
    if (fabs(dq_d) > MOTOR_VEL_MAX*SAFETY_FACTOR){ // 21rad/s * 0.8 = 16.8
        cout << "[Motor " << _motor_s.id << "]: dq_d safety error\n";
        return false;
    } 
    // if (fabs(q_d) >= M_PI){ 
    //     cout << "[Motor " << _motor_s.id << "]: q_d safety error\n";
    //     return false;
    // } 
    if (kp > 5 || kp < 0){
        cout << "[Motor " << _motor_s.id << "]: kp safety error\n";
        return false;
    } 
    if (kd > 10 || kd < 0){
        cout << "[Motor " << _motor_s.id << "]: kd safety error\n";
        return false;
    }
    return true;
}

void MotorControl::_extract()
{
    extract_data(&_motor_r); // decode
    temp = _motor_r.Temp;
    q = (_motor_r.Pos / GEAR_RATIO) - _offset;
    dq = _motor_r.LW / GEAR_RATIO; // with filter 
    tau = _motor_r.T * GEAR_RATIO;
}

bool MotorControl::_send_recv()
{
    _sta = _serial_ptr->sendRecv(&_motor_s, &_motor_r);
    if (!_sta){ // 通讯失败
        cout << "[Motor " << _motor_s.id << "]: send_recv error\n";
        return false;
    }
    return true;
}

bool MotorControl::stop()
{
    _motor_s.mode = 0; // stop mode
    modify_data(&_motor_s); 
    if (!this->_send_recv()) {return false;}
    this->_extract();
    return true;
}

bool MotorControl::run(double tau_d, double dq_d, double q_d, double kp, double kd)
{
    if (this->_cmd_is_safe(tau_d, dq_d, q_d, kp, kd)){
        _motor_s.mode = 10; // servo mode
        _motor_s.T = tau_d / GEAR_RATIO;
        _motor_s.W = dq_d * GEAR_RATIO;
        _motor_s.Pos = (q_d + _offset) * GEAR_RATIO;
        _motor_s.K_P = kp; // K_P < 63.9
        _motor_s.K_W = kd; // K_W < 31.9
        modify_data(&_motor_s);
        if (!this->_send_recv()) {return false;}
        this->_extract();
        return true;
    }
    return false;
}

bool MotorControl::run(double tau_d)
{
    return this->run(tau_d, 0.0, 0.0, 0.0, 0.0);
}

std::ostream & operator<<(std::ostream &os, const MotorControl &motor)
{
    os << "[Motor id]: " << (int)motor._motor_r.motor_id << endl  // 0,1,2
        << "[Motor temperature]: " << motor.temp << " ℃" << endl
        << "[Motor torque]: " << motor.tau << " Nm" << endl
        << "[Motor velocity]: " << motor.dq << " rad/s" << endl
        << "[Motor position]: " << motor.q * (180.0 / M_PI) << " deg" << endl;
    return os;
}


/*
db      d88888b  d888b  
88      88'     88' Y8b 
88      88ooooo 88      
88      88~~~~~ 88  ooo 
88booo. 88.     88. ~8~ 
Y88888P Y88888P  Y888P  
*/
LegControl::LegControl(SerialPort* serial_ptr, vector<double> offset_vec, LegType leg)
{
    this->_serial_ptr = serial_ptr;
    this->_offset_vec = offset_vec;
    this->leg_type = leg;
    switch (this->leg_type)
    {
        case LegType::FR:
            _pos_mapping << 1., 0., 0.,
                            0., 0., -1.,
                            0., -1., 1.;
            _pos_inv_mapping << 1., 0., 0.,
                                0., -1., -1., 
                                0., -1., 0.;
            break;
        case LegType::FL:
            _pos_mapping << 1., 0., 0.,
                            0., 1., 0.,
                            0., -1., 1.;
            _pos_inv_mapping << 1., 0., 0.,
                                0., 1., 0.,
                                0., 1., 1.;
            break;
        case LegType::RR:
            _pos_mapping << -1., 0., 0.,
                            0.,  0., -1.,
                            0., -1., 1.;
            _pos_inv_mapping << -1., 0., 0.,
                                0., -1., -1.,
                                0., -1, 0.;
            break;
        case LegType::RL:
            _pos_mapping << -1., 0., 0., 
                            0., 1., 0., 
                            0., -1., 1.;
            _pos_inv_mapping << -1., 0., 0.,
                                0., 1., 0.,
                                0., 1., 1.;
            break;
    }
    // Linear mapping -> pos mapping = vel mapping
    _vel_mapping = _pos_mapping;
    _vel_inv_mapping = _pos_inv_mapping;

    this->_motors = vector<MotorControl*>(3);
    this->_motors[0] = new MotorControl(this->_serial_ptr, 0, this->_offset_vec[0]);
    this->_motors[1] = new MotorControl(this->_serial_ptr, 1, this->_offset_vec[1]);
    this->_motors[2] = new MotorControl(this->_serial_ptr, 2, this->_offset_vec[2]);
    // pubulic member
    temp_vec = vector<double>(3, 0.);
    tau.setZero();
    dq.setZero();
    q.setZero();
    // init leg states
    this->stop();
}

LegControl::~LegControl()
{
    for (auto motor_ptr : _motors) {delete motor_ptr;}
}

bool LegControl::_cmd_is_safe(Vector3d q_d)
{
    // phi0
    if ((leg_type == LegType::FL) || (leg_type == LegType::RL)) {
        if ((q_d(0) < X_ALONG_ANGLE_MIN) || (q_d(0) > X_ALONG_ANGLE_MAX)) {
            cout << "[Leg" << (int)leg_type << "]: joint 0 pos safety error(0)\n"; 
            return false;
        }
    }
    else {
        if ((q_d(0) < -X_ALONG_ANGLE_MAX) || (q_d(0) > -X_ALONG_ANGLE_MIN)) {
            cout << "[Leg" << (int)leg_type << "]: joint pos safety error(0)\n"; 
            return false;
        }
    }
    // phi1 and phi2
    if ((fabs(q_d(1)) > Y_ALONG_ANGLE_LIMIT) || (fabs(q_d(2)) > Y_ALONG_ANGLE_LIMIT)) {
        cout << "[Leg" << (int)leg_type << "]: joint pos safety error(1)\n"; 
        return false;
    }
    // eta = phi2 - phi1
    if (((q_d(2)-q_d(1)) > ETA_MAX) || ((q_d(2)-q_d(1)) < ETA_MIN)) {
        cout << "[Leg" << (int)leg_type << "]: joint pos safety error(2)\n";  
        return false;
    }

    return true;
}

void LegControl::_extract()
{
    Vector3d tau_motor, dq_motor, q_motor;
    // update motor states
    for (int i = 0; i < 3; i++){
        temp_vec[i] = _motors[i]->temp;
        tau_motor(i) = _motors[i]->tau;
        dq_motor(i) = _motors[i]->dq;
        q_motor(i) = _motors[i]->q;
    }
    // from actual motor states to theoretical motor states
    q = _pos_inv_mapping * q_motor;
    dq = _vel_inv_mapping * dq_motor;
    tau = _vel_mapping.transpose() * tau_motor;
}

bool LegControl::stop()
{
    int success = 0;
    for (auto motor_ptr : _motors){
        if (motor_ptr->stop()) {++success;}
    }
    this->_extract();
    if (success != 3) {return false;}
    return true;
}

bool LegControl::run(const Vector3d &tau_d, const Vector3d &dq_d, const Vector3d &q_d,
                     Vector3d kp, Vector3d kd)
{
    Vector3d tau_motor, dq_motor, q_motor;
    q_motor = _pos_mapping * q_d;
    dq_motor = _vel_mapping * dq_d;
    tau_motor = _vel_inv_mapping.transpose() * tau_d;
    int success = 0;
    if (_cmd_is_safe(q_d)) {
        for (int i = 0; i < 3; i++) {
            if (_motors[i]->run(tau_motor(i), dq_motor(i), q_motor(i), kp(i), kd(i))) 
            {++success;}
        }
        this->_extract();
        if (success != 3) {return false;}
        return true;
    }
    return false;
}

bool LegControl::run(const Vector3d &tau_d)
{
    Vector3d dq_d, q_d;
    dq_d.setZero();
    q_d.setZero();
    return this->run(tau_d, dq_d, q_d, Vector3d::Zero(), Vector3d::Zero());
}

std::ostream & operator<<(std::ostream &os, const LegControl &leg)
{
    os << "[Leg id]: " << (int)leg.leg_type << endl  // 0,1,2,3
        << "[Motor temperature]: " << leg.temp_vec[0] << " " << leg.temp_vec[1] << " " << leg.temp_vec[2] << endl 
        << "[Motor torque]: " << leg.tau.transpose() << endl
        << "[Motor velocity]: " << leg.dq.transpose() << endl
        << "[Motor position]: " << leg.q.transpose() << endl;
    return os;
}
