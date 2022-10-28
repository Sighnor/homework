#ifndef Strategy_H
#define Strategy_H
//插值
float Clamp(const float& a, const float& b, float t)
{
    return (1.0 - t) * a + t * b;
}
//下降单纯形法
template<int DIM, typename FUNC>
float DownHill(float* start, FUNC &objectiveFn)
{
    size_t highest;
    size_t next;
    size_t lowest;
    float F[DIM];
    float P[DIM];

    for (size_t i = 0;i < DIM;i++)//对每个点计算对应值
    {
        P[i] = start[i];
        F[i] = objectiveFn(start[i]);
        //std::cout << P[i] << ',' << F[i] << std::endl;
    }

    for (int j = 0;j < 100;j++)//迭代
    {
        //寻找最高，最低，次低点
        highest = lowest = 0;
        for (size_t i = 0;i < DIM;i++)
        {
            if (F[i] >= F[highest])
            {
                highest = i;
            }
            else if (F[i] <= F[lowest])
            {
                lowest = i;
            }
        }
        next = highest;
        for (size_t i = 0;i < DIM;i++)
        {
            if (F[i] > F[lowest])
            {
                if (F[i] < F[next])
                {
                    next = i;
                }
            }
        }
        //std::cout << "now: " << P[highest] << ',' << F[highest] << std::endl;
        //std::cout << "now: " << P[lowest] << ',' << F[lowest] << std::endl;
        //std::cout << lowest << ',' << next << ',' << highest << std::endl;

        if((F[highest] - F[lowest]) < 0.0001f)//ֹͣ停止迭代
        {
            //std::cout << (F[highest] - F[lowest]) << ',' << "break!" << std::endl;
            break;
        }
        //除去最差点的平均值
        float center = 0.0;
        for (size_t i = 0;i < DIM;i++)
        {
            if (i == lowest)
            {
                continue;
            }
            center += P[i] / (DIM - 1);
        }
        //反射
        float P_reflect = Clamp(center, P[lowest], -1.0);
        float F_reflect = objectiveFn(P_reflect);
        if (F_reflect >= F[next] && F_reflect <= F[highest])//反射更优
        {
            //取代最低点
            P[lowest] = P_reflect;
            F[lowest] = F_reflect;
            std::cout << "reflect " << P_reflect<< ',' << F_reflect << std::endl;
        }
        else if (F_reflect > F[highest])//反射最优
        {
            //扩张
            float P_expand = Clamp(center, P[lowest], -2.0);
            float F_expand = objectiveFn(P_expand);
            if (F_expand < F_reflect)//扩张不如反射
            {
                //取代最低点
                P[lowest] = P_reflect;
                F[lowest] = F_reflect;
                std::cout << "reflect " << P_reflect<< ',' << F_reflect << std::endl;
            }
            else//反射不如扩张
            {
                //取代最低点
                P[lowest] = P_expand;
                F[lowest] = F_expand;
                std::cout << "expand " << P_expand<< ',' << F_expand << std::endl;
            }
        }
        else if (F_reflect < F[next])//反射比次差点要差
        {
            float t;
            F_reflect > F[lowest] ? t = -0.5 : t = 0.5;//选择收缩方向
            float P_contract = Clamp(center, P[lowest], t);
            float F_contract = objectiveFn(P_contract);
            if (F_contract > F_reflect)//收缩优于反射
            {
                //取代最低点
                P[lowest] = P_contract;
                F[lowest] = F_contract;
                std::cout << "contract " << P_contract << ',' << F_contract << std::endl;
            }
            else//收缩差于反射
            {
                for (size_t i = 0;i < DIM;i++)//除最高点整体收缩
                {
                    if (i == highest)
                    {
                        continue;
                    }
                    P[i] = Clamp(P[i], P[highest], 0.2);
                    F[i] = objectiveFn(P[i]);
                    std::cout << "shrink " << std::endl;
                }
            }
        }
    }
    return P[highest];//返回最高点
}

template<typename FUNC>
float ClimbHill(float start, FUNC &objectiveFn)
{
    /*��Ե*/
    const float limit_a = 1+0.8;
    const float limit_b = 30-0.8;

    /*��ɽ�����йر���*/
    int direction = 1;//���򣬷���Ϊ-1
    const float step_init = 1;//��ʼ����
    float step = step_init;//��ǰ����
    const float step_factor = 0.3;//�����仯��
    const int num_moves_max = 5;//����ƶ�����
    int num_moves_now = 0;//��ǰ�ƶ�����
    const float limit_neg_diff = -0.7;//������ֵ��
    const float limit_abs_diff = 0.3;//����ֵ����ֵ

    /*�Խ��йر���*/
    float focus_top = start;//������󣬵�ǰ����һ����
    float focus_now = start;
    float focus_next = 0;
    float difference_top = objectiveFn(start);//��󣬵�ǰ����һ����
    float difference_now = objectiveFn(start);
    float difference_next = 0;


    do
    {
        std::cout << focus_now << "   " << difference_now << std::endl;
        focus_next = focus_now + step * direction;//����ǰ�����ƶ�
            
        if (focus_next < limit_a || focus_next > limit_b)//��������Ե
        {
            direction *= -1;//����
            focus_next = focus_now + step * direction;
            step = step_init;
        }

        difference_next = objectiveFn(focus_next);//���㷴��ֵ
        num_moves_now++;

        if (difference_next > difference_top)//�������ֵ������󷴲�ֵ
        {
            difference_top = difference_next;
            focus_top = focus_next;

            num_moves_now = 0;
            step = step_init;
        }

        if (difference_next - difference_now < limit_neg_diff)
        {
            direction *= -1;
            step *= 1 - step_factor;//��С����
        }
        else if (abs(difference_next - difference_now) < limit_abs_diff)
        {
            step *= 1 + step_factor;//���󲽳�
        }

        focus_now = focus_next;
        difference_now = difference_next;

    } while (num_moves_now < num_moves_max);


    return focus_top;
}

#endif 
