#include <iostream>
#include <bits/stdc++.h>
#include <easyx.h>
#include <conio.h>
#include<random>
#include <fstream>
using namespace std;
#define PI 3.1415926
struct State {
	double x;
	double y;
	double yaw;
	double v;
	double w;
};
struct Config {
	//最大速度和最小速度 单位暂定为像素
	double v_min = 0;
	double v_max = 20;
	//最大角速度和最小角速度 单位为弧度制rad/s
	double w_min = -40 * PI / 180;
	double w_max = 40 * PI / 180;
	//最大线加速度角加速度 减速度和加速度一样
	double va_max = 2;
	double wa_max = 40 * PI / 180;
	//分辨率
	double v_sample = 0.1;
	double w_sample = 5 * PI / 180;
	//时间片
	double dt = 0.5;
	//预测时间
	double p_time = 2;
	//起点终点
	State start = { 50,50,-PI,0,0 };
	pair<double, double> end = { 500,500 };
	//障碍物
	vector<pair<double, double>> static_obs, dynamic_obs;
	//动态障碍物速度 每dt时间
	double dynamic_obs_v = 1.5;
	//船体半径
	double radius = 30;
	//评价直播增益系数
	double alpha = 1;
	double beta = 1;
	double gamma = 1;
}config;
struct Space {
	double v1;
	double v2;
	double w1;
	double w2;
};
//以特定速度空间内 运动下一时刻dt的状态
State motion(State state, pair<double, double> vw)
{
	double v = vw.first, w = vw.second;
	state.x += v * cos(state.yaw) * config.dt;//以v速度前进
	state.y += v * sin(state.yaw) * config.dt;
	state.yaw += w * config.dt;//以w角速度转角
	state.v = v;
	state.w = w;
	return state;
}
//计算dt内速度空间
Space v_space_count(State state)
{
	pair<double, double> v_vlim, a_vlim;
	v_vlim.first = config.v_min;
	v_vlim.second = config.v_max;
	a_vlim.first = state.v - config.va_max * config.dt;//最大加速度加速
	a_vlim.second = state.v + config.va_max * config.dt;//最大加速度减速

	pair<double, double> v_wlim, a_wlim;
	v_wlim.first = config.w_min;
	v_wlim.second = config.w_max;
	a_wlim.first = state.w - config.wa_max * config.dt;
	a_wlim.second = state.w + config.wa_max * config.dt;

	Space space;
	space.v1 = max(v_vlim.first, a_vlim.first);
	space.v2 = min(v_vlim.second, a_vlim.second);
	space.w1 = max(v_wlim.first, a_wlim.first);
	space.w2 = min(v_wlim.second, a_wlim.second);
	return space;
}
//轨迹预测
vector<State> predict_trajectory(State state, pair<double, double> vw)
{
	double v = vw.first, w = vw.second;
	vector<State> trajectory;
	trajectory.push_back(state);
	for (double i = 0; i <= config.p_time; i += config.dt)
	{
		state = motion(state, vw);
		trajectory.push_back(state);
	}
	return trajectory;
}
//轨迹评价
double evaluate_trajectory(vector<State> trajectory)
{
	pair<double, double> point = { trajectory[trajectory.size() - 1].x,trajectory[trajectory.size() - 1].y };
	//heading
	double dx = config.end.first - point.first;
	double dy = config.end.second - point.second;
	double a1 = atan2(dy, dx);
	double heading = PI - abs(trajectory[trajectory.size() - 1].yaw - a1);
	//velocity
	double velocity = trajectory[trajectory.size() - 1].v;
	//obs
	double dist = 9999;
	for (int i = 0; i < config.static_obs.size(); i++)
	{
		double temp = hypot(point.first - config.static_obs[i].first, point.second - config.static_obs[i].second);
		if (temp < dist)
		{
			dist = temp;
		}
	}
	for (int i = 0; i < config.dynamic_obs.size(); i++)
	{
		double temp = hypot(point.first - config.dynamic_obs[i].first, point.second - config.dynamic_obs[i].second);
		if (temp < dist)
		{
			dist = temp;
		}
	}
	if (dist > config.radius * 1.5)//如果最近的障碍物距离大于1.5倍半径，设置为此方向无障碍物
	{
		dist = 100;
	}
	return heading * config.alpha + velocity * config.beta + dist * config.gamma;
}
//绘制轨迹
void draw_trajectory(vector<State> trajectory)
{
	setfillcolor(RED);
	setlinecolor(RED);
	for (int i = 0; i < trajectory.size() - 1; i++)
	{
		line(trajectory[i].x, trajectory[i].y, trajectory[i + 1].x, trajectory[i + 1].y);
	}
}
void init_obs()
{
	//放置静态障碍物
	config.static_obs.push_back({ 100,100 });
	config.static_obs.push_back({ 100,200 });
	config.static_obs.push_back({ 400,350 });
	config.static_obs.push_back({ 400,400 });
	config.static_obs.push_back({ 400,450 });
	config.static_obs.push_back({ 400,500 });
	//放置动态障碍物
	config.dynamic_obs.push_back({ 300,300 });//走斜直线障碍物
	config.dynamic_obs.push_back({ 200,100 });//下走
	config.dynamic_obs.push_back({ 300,100 });//下走
	config.dynamic_obs.push_back({ 200,300 });//右走
	config.dynamic_obs.push_back({ 200,400 });//右走
	//config.dynamic_obs.push_back({ 400,400 });//走圆形障碍物
}
void drawinit()
{
	initgraph(600, 600);
	setbkcolor(WHITE);
	cleardevice();
	setfillcolor(BLACK);
	setlinecolor(BLACK);
	fillcircle(config.start.x, config.start.y, 5);//起点
	setfillcolor(RED);
	setlinecolor(RED);
	fillcircle(config.end.first, config.end.second, 5);//终点
	//静态障碍物
	setfillcolor(BLACK);
	setlinecolor(BLACK);
	for (int i = 0; i < config.static_obs.size(); i++)
	{
		fillcircle(config.static_obs[i].first, config.static_obs[i].second, config.radius);
	}
	for (int i = 0; i < config.dynamic_obs.size(); i++)
	{
		fillcircle(config.dynamic_obs[i].first, config.dynamic_obs[i].second, config.radius);
	}
	//动态障碍物放在obs_change内进行更新
}
void obs_change()
{
	setfillcolor(WHITE);
	setlinecolor(WHITE);
	for (int i = 0; i < config.dynamic_obs.size(); i++)
	{
		fillcircle(config.dynamic_obs[i].first, config.dynamic_obs[i].second, config.radius);
	}
	double dy = 100 - 300, dx = 500 - 200;
	double angle = atan2(dy, dx);
	int flag = 1;
	//如果非常接近右端点
	if (hypot(500 - config.dynamic_obs[0].first, 100 - config.dynamic_obs[0].second) < 10)
	{
		flag = 0;
	}
	if (hypot(500 - config.dynamic_obs[0].first, 100 - config.dynamic_obs[0].second) > 360)
	{
		flag = 1;
	}
	if (flag == 1)
	{
		config.dynamic_obs[0].first += config.dynamic_obs_v * cos(angle);
		config.dynamic_obs[0].second += config.dynamic_obs_v * sin(angle);
	}
	else
	{
		config.dynamic_obs[0].first -= config.dynamic_obs_v * cos(angle);
		config.dynamic_obs[0].second -= config.dynamic_obs_v * sin(angle);
	}
	for (int i = 1; i <=2; i++)
	{
		config.dynamic_obs[i].second += config.dynamic_obs_v;
	}
	for (int i = 3; i <= 4; i++)
	{
		config.dynamic_obs[i].first += config.dynamic_obs_v;
	}
	setfillcolor(BLACK);
	setlinecolor(BLACK);
	for (int i = 0; i < config.dynamic_obs.size(); i++)
	{
		fillcircle(config.dynamic_obs[i].first, config.dynamic_obs[i].second, config.radius);
	}

	
}
int main()
{
	init_obs();
	drawinit();
	State state = config.start;
	int flag = 1;
	pair<double, double> end_vw;
	//system("pause");
	while (true)
	{
		if (flag == 1)
		{
			Space space = v_space_count(state);
			double score_max = -1;
			vector<State> trajectory_good;
			pair<double, double> vw_good;
			for (double i = space.v1; i <= space.v2; i += config.v_sample)//遍历速度空间
			{
				for (double j = space.w1; j <= space.w2; j += config.w_sample)
				{
					pair<double, double> ij = { i,j };
					vector<State> trajectory = predict_trajectory(state, ij);
					//draw_trajectory(trajectory);
					double score = evaluate_trajectory(trajectory);
					if (score > score_max)
					{
						score_max = score;
						trajectory_good = trajectory;
						vw_good = ij;
					}
					//system("pause");
				}
			}

			draw_trajectory(trajectory_good);
			state = motion(state, vw_good);
			obs_change();
			//cout << vw_good.first << " " << vw_good.second << endl;
			/*if (vw_good.first == 0)
			{
				cout << "控制速度目前为0" << endl;
			}*/
			//system("pause");
			setfillcolor(GREEN);
			setlinecolor(GREEN);
			fillcircle(state.x, state.y, 2);
			if (hypot(trajectory_good[trajectory_good.size() - 1].x - config.end.first, trajectory_good[trajectory_good.size() - 1].y - config.end.second) <= config.radius / 2)
			{
				//说明已经找到最优路径了
				cout << "find" << endl;
				flag = 0;
				end_vw = vw_good;
			}
			Sleep(50);
		}
		else
		{
			state = motion(state, end_vw);
			obs_change();
			setfillcolor(GREEN);
			setlinecolor(GREEN);
			fillcircle(state.x, state.y, 2);
			if (hypot(state.x - config.end.first, state.y - config.end.second) < config.radius / 2)
			{
				cout << "到达" << endl;
				break;
			}
			Sleep(50);
		}

	}
	_getch();
	closegraph();
}
