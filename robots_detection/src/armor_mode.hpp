#ifndef ARMOR_MODE
#define ARMOR_MODE
#include <iostream>
#include <deque>
#include <vector>
enum MOTION_MODE
{
    CIRCLE,
    TWIST,
    NORMAL,
    CSPEED
};
enum STATUS
{
    REMAIN,
    RETAIN,
    RENEW
};
class Deque_mode
{
  public:
    Deque_mode() : max_size(50) {}
    std::deque<int> deque_mode;
    size_t max_size;
    std::deque<size_t> deque_circle;
    std::deque<size_t> deque_twist;
    void push(float &angle)
    {
        if (deque_mode.size() == max_size)
        {
            deque_mode.pop_front();
        }
        if (angle > 0.001)
        {
            if (angle > 0.085)
            {
                this->deque_mode.push_back(2);
            }
            else
            {
                this->deque_mode.push_back(1);
            }
        }
        else if (angle < -0.001)
        {
            if (angle < -0.085)
            {
                this->deque_mode.push_back(-2);
            }
            else
            {
                this->deque_mode.push_back(-1);
            }
        }
        else
        {
            this->deque_mode.push_back(0);
        }
    }
    void circle_mode()
    {
        this->deque_circle.clear();
        int begin = -1;
        for (int i = 0; i < this->deque_mode.size(); ++i)
        {
            if (abs(this->deque_mode[i]) == STATUS::RENEW)
            {
                if (begin == -1)
                {
                    begin = i;
                }
                else
                {
                    if (i - begin)
                    {
                        this->deque_circle.push_back(i - begin);
                    }
                    begin = i;
                }
            }
        }
        if (this->deque_circle.empty())
        {
            return;
        }
        else
        {
            float mean = 0;
            for (float mode : this->deque_circle)
            {
                mean += mode;
            }
            mean = mean / this->deque_circle.size();
            float variance = 0;
            for (size_t mode : this->deque_circle)
            {
                variance += (mean - mode) * (mean - mode);
            }
            variance = sqrt(variance / (this->deque_circle.size() - 1));
            std::cout << "CIRCLE\tVARRIANCE: " << variance << std::endl;
        }
    }
    void twist_mode()
    {
        this->deque_twist.clear();
        size_t begin = 0;
        for (int i = 1; i < this->deque_mode.size(); ++i)
        {
            if (this->deque_mode[i - 1] * this->deque_mode[i] < 0)
            {
                if (begin)
                {
                    this->deque_twist.push_back(i - begin);
                    begin = i;
                }
                else
                {
                    begin = i;
                }
            }
        }
        if (this->deque_twist.empty())
        {
            std::cout << "TWIST!\tNO MEAN OCCURE!" << std::endl;
            return;
        }
        else
        {
            float mean = 0;
            for (float mode : this->deque_twist)
            {
                mean += mode;
            }
            mean = mean / this->deque_twist.size();
            std::cout << "TWIST!\t MEAN: " << mean << std::endl;
            float variance = 0;
            for (size_t mode : this->deque_twist)
            {
                variance += (mean - mode) * (mean - mode);
            }
            variance = sqrt(variance / (this->deque_twist.size() - 1));
            std::cout << "TWIST\tVARIANCE: " << variance << std::endl;
        }
    }
};
#endif