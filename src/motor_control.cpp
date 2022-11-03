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
    this->_motor_s.motorType = MotorType::A1Go1;
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
    if (fabs(q_d) >= 2*M_PI){ // TODO: change
        cout << "[Motor " << _motor_s.id << "]: q_d safety error\n";
        return false;
    } 
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
    os << "[Motor id]: " << motor._motor_r.motor_id << endl  // 0,1,2
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
// LegControl::LegControl(SerialPort* serial_ptr, vector<double> offset_list, LegType leg)
// {
//     // private menber
//     this->serial_ptr = serial_ptr;
//     this->offset_list = offset_list;
//     this->motors = vector<MotorControl*>(3);
//     this->motors[0] = new MotorControl(this->serial_ptr, 0, this->offset_list[0]);
//     this->motors[1] = new MotorControl(this->serial_ptr, 1, this->offset_list[1]);
//     this->motors[2] = new MotorControl(this->serial_ptr, 2, this->offset_list[2]);
//     this->leg_type = leg;
//     this->kine_ptr = new LegKinematicsBennett(this->leg_type);
//     switch (this->leg_type)
//     {
//         case LegType::FR:
//             axis_transfer << 1., -1., -1.;
//             break;
//         case LegType::FL:
//             axis_transfer << 1., 1., 1.;
//             break;
//         case LegType::RR:
//             axis_transfer << -1., -1., -1.;
//             break;
//         case LegType::RL:
//             axis_transfer << -1., 1., 1.;
//             break;
//     }
//     // pubulic member
//     temp_list = vector<double>(3, 0.);
//     tau.setZero();
//     dq.setZero();
//     q.setZero();
//     foot_force.setZero(); 
//     foot_vel.setZero(); 
//     foot_pos.setZero();
//     jacobian.setIdentity();
//     // init leg states
//     this->stop();
// }

// LegControl::~LegControl()
// {
//     for (auto motor_ptr : motors) {delete motor_ptr;}
//     delete kine_ptr;
// }

// bool LegControl::cmd_is_safe(Vector3d q_d)
// {
//     // phi0
//     if ((leg_type == LegType::FL) || (leg_type == LegType::RL)) 
//     {
//         if ((q_d(0) < -10./180.*M_PI) || (q_d(0) > 90./180.*M_PI)) 
//         {
//             cout << "[Leg" << (int)leg_type << " cmd]: phi0 safety error\n"; 
//             return false;
//         }
//     }
//     else // LegType::FR || LegType::RR
//     {
//         if ((q_d(0) < -90./180.*M_PI) || (q_d(0) > 10./180.*M_PI)) 
//         {
//             cout << "[Leg" << (int)leg_type << " cmd]: phi0 safety error\n"; 
//             return false;
//         }
//     }
//     // phi1
//     if (fabs(q_d(1)) > 80./180.*M_PI) 
//     {
//         cout << "[Leg" << (int)leg_type << " cmd]: phi1 safety error\n"; 
//         return false;
//     }
//     // phi2
//     if (fabs(q_d(1)+q_d(2)) > 80./180.*M_PI) 
//     {
//         cout << "[Leg" << (int)leg_type << " cmd]: phi2 safety error\n"; 
//         return false;
//     }
//     // eta
//     if ((q_d(2) > ETA_MAX) || (q_d(2) < ETA_MIN)) 
//     {
//         cout << "[Leg" << (int)leg_type << " cmd]: eta safety error\n"; 
//         return false;
//     }

//     return true;
// }

// void LegControl::extract()
// {
//     // update motor states
//     for (int i = 0; i < 3; i++)
//     {
//         temp_list[i] = motors[i]->temp;
//         // from actual motor states to theoretical motor states
//         tau(i) = axis_transfer(i) * motors[i]->tau;
//         dq(i) = axis_transfer(i) * motors[i]->dq;
//         q(i) = axis_transfer(i) * motors[i]->q;
//     }
//     // update foot states
//     foot_pos = kine_ptr->FK_motor(q).block<3,1>(0,3);
//     jacobian = kine_ptr->Jacobian_motor(q);
//     foot_vel = jacobian * dq;
//     foot_force = jacobian.inverse() * tau;
// }

// bool LegControl::stop()
// {
//     for (auto motor_ptr : motors)
//     {
//         if (!motor_ptr->stop()) {return false;}
//     }
//     this->extract();
//     return true;
// }

// bool LegControl::run(Vector3d tau_d, Vector3d dq_d, Vector3d q_d, Vector3d kp, Vector3d kd)
// {
//     // from actual motor states to theoretical motor states
//     if (cmd_is_safe(q_d)) {
//         for (int i = 0; i < 3; i++) {
//             if (!motors[i]->run(axis_transfer(i)*tau_d(i),
//                                 axis_transfer(i)*dq_d(i),
//                                 axis_transfer(i)*q_d(i),
//                                 kp(i),
//                                 kd(i))) 
//             {return false;}
//         }
//         this->extract();
//         return true;
//     }
//     return false;
// }

// bool LegControl::run(Vector3d tau_d)
// {
//     for (int i = 0; i < 3; i++) {
//         if (!motors[i]->run(axis_transfer(i)*tau_d(i))) 
//         {return false;}
//     }
//     this->extract();
//     return true;
// }

// std::ostream & operator<<(std::ostream &os, const LegControl &leg)
// {
//     os << "[Leg id]: " << (int)leg.leg_type << endl  // 0,1,2,3
//         << "[Motor temperature]: " << leg.temp_list[0] << " " << leg.temp_list[1] << " " << leg.temp_list[2] << endl 
//         << "[Leg force]: " << leg.foot_force.transpose() << endl
//         << "[Leg velocity]: " << leg.foot_vel.transpose() << endl
//         << "[Leg position]: " << leg.foot_pos.transpose() << endl
//         << "[Motor torque]: " << leg.tau.transpose() << endl
//         << "[Motor velocity]: " << leg.dq.transpose() << endl
//         << "[Motor position]: " << leg.q.transpose() << endl;
//     return os;
// }
