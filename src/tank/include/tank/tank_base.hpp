/**
 * @file tank_base.h
 * @author North.D.K (north24@qq.com)
 * @brief plugin base for tank model, all actions start here
 * @version 0.1
 * @date 2021-11-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef TANK_BASE_HPP
#define TANK_BASE_HPP
#include <string>

class Tank
{
public:
    virtual void init(const std::string& name) = 0;
    virtual void action() = 0;
    // virtual ~Tank() {}

protected:
    Tank(){}
};
#endif