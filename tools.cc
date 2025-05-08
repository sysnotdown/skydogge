
#include "tools.h"
#include <math.h>

// void TrimString(std::string& str)
// {
// 	std::string::size_type pos = str.find_last_not_of(" \r\n");

// 	if (pos != std::string::npos)
// 	{
// 		str.erase(pos + 1);
// 		pos = str.find_first_not_of(" \r\n");

// 		if (pos != std::string::npos)
// 			str.erase(0, pos);
// 	}
// 	else
// 		str.erase(str.begin(), str.end());
// }

void TrimString(std::string& str)
{
		std::string::size_type pos = str.find_last_not_of(' ');

		if (pos != std::string::npos)
		{
			str.erase(pos + 1);
			pos = str.find_first_not_of(' ');

			if (pos != std::string::npos)
				str.erase(0, pos);
		}
		else
			str.erase(str.begin(), str.end());
}

//items=a;b;c;d...
//检查通过
int ParseTaskList(std::string items, std::vector<std::string>& splits)
{

	size_t pos;

	do
	{
		TrimString(items);

		pos = items.find(';');

		if (pos == items.npos) {

			if (!items.empty())
			{
				splits.push_back(items);
				return splits.size();
			}
			else
			{
				return splits.size();
			}
		}
		else {
			std::string ns = items.substr(0, pos);
			items.erase(0, pos + 1);
			TrimString(ns);
			if (!ns.empty()) {
				splits.push_back(ns);
			}
		}

	} while (1);
}

int ParseLine(std::string& lines, std::vector<std::string>& splits)
{
	bool quoted = false;
	size_t tokenStart = 0;

	for (size_t i=0; i < lines.size(); i++)
	{
		if (lines[i] == '"')
			quoted = !quoted;
		else if (lines[i] == ',' && !quoted)
		{
			splits.push_back(lines.substr(tokenStart, i - tokenStart));
			tokenStart = i + 1;
		}
	}

	//end
	splits.push_back(lines.substr(tokenStart, lines.size() - tokenStart));
	return splits.size();
}

//分号切割任务
int SplitLine(std::string& lines, std::vector<std::string>& splits)
{
	bool quoted = false;
	size_t tokenStart = 0;

	for (size_t i=0; i < lines.size(); i++)
	{
		if (lines[i] == '"')
			quoted = !quoted;
		else if (lines[i] == ';' && !quoted)
		{
			splits.push_back(lines.substr(tokenStart, i - tokenStart));
			tokenStart = i + 1;
		}
	}

	//end
	splits.push_back(lines.substr(tokenStart, lines.size() - tokenStart));
	return splits.size();
}

//items=(a,b,c...), 括号外可以有内容。
int ParseFloatList(std::string items, std::vector<float>& vals)
{
		size_t pos5 = items.find_first_of('(');
		size_t pos6 = items.find_last_of(')');

		if (pos5 == std::string::npos || pos6 == std::string::npos) return 0;

		std::string str = items.substr(pos5 + 1, pos6 - pos5 - 1); //去括号

		size_t pos;

		do
		{
			TrimString(str);

			pos = str.find(',');

			if (pos == str.npos) {

				if (!str.empty())
				{
					float k = atof(str.c_str());
					vals.push_back(k);
					return vals.size();
				}
				else
				{
					return vals.size();
				}
			}
			else {
				std::string ns = str.substr(0, pos);
				str.erase(0, pos + 1);
				TrimString(ns);
				if (!ns.empty()) {
					float k = atof(ns.c_str());
					vals.push_back(k);
				}
			}

		} while (1);
	
	return vals.size();
}

//坐标轴xy体系下，和y轴正方向的夹角，0-360度。

float heading_angle(float x, float y)
{
	float heading;
	//0-360的航向角，顺时针。
	if (x >= 0 && y >= 0)
	{
		//0-90
		heading = atan2(x, y)*57.2958; //度数。如果cx绝对值极大，则x轴越靠近北方。返回值接近90度。
	}
	else if (x >= 0 && y < 0)
	{
		//90-180
		heading = 180- atan2(x, -y)*57.2958;
	}
	else if (x < 0 && y < 0)
	{
		//180-270
		heading = 180 + atan2(-x, -y)*57.2958;
	}
	else
	{
       //270-360.
		heading = 360 - atan2(-x, y)*57.2958;
	}

	return heading;
}



//因为要计算磁偏角。所以需要一个计算全球距离及角度的代码。
//没有找到合适的，自己写一个。
//要放在三维空间来考虑这个问题，
//建立坐标系，地心指向0经度线和赤道的交点为正向X轴。
//地心指向东经90度和赤道的交点为正向Y轴。
//地心指向北极为Z轴正向。东经0-180度是X轴的上半平面，西经0-180是下半平面。
//为了统一计算，东经的值直接输入函数，西经的值用360-西经值，这样输入的经度范围是0-360度。
//西经采用负值算出来结果一样。如西经112度，输入-112或者 360-112结果一样。
//北纬的值直接输入函数，南纬的值输入负值(-南纬)，这样纬度的范围统一为-90-90度。
//在这个三维坐标系里，利用给出的纬度经度值，可以计算出三个坐标点。
//坐标点之间的直线连接必定是大圆的一个弦长。利用弦长可计算夹角，夹角可计算大圆弧长。
//两个弦长之间的夹角也可以利用矢量方法计算。
//返回的距离是大圆弧长，角度是起点到北极的角和到目的地的角之间的夹角。
//西经和南纬需要转换，东经北纬不需要。
//全球大范围计算距离及航向角。东经北纬原样输入，西经转为360-西经，南纬转负值
//全球大范围计算距离及航向角。
//计算时间大约131us, 简化版的大约42us，完全采用double计算似乎时间也一样。

void global_gps_dist_angle(double lati0, double logi0, double lati1, double logi1, float& dist, float& angle)
{
#define ER 6380000
#define cratio  57.29578
	//纬度决定z轴数值。
	float z0 = sin(lati0 / cratio);
	float r0 = cos(lati0 / cratio);
	float x0 = r0 * cos(logi0 / cratio);
	float y0 = r0 * sin(logi0 / cratio);

	float z1 = sin(lati1 / cratio);
	float r1 = cos(lati1 / cratio);
	float x1 = r1 * cos(logi1 / cratio);
	float y1 = r1 * sin(logi1 / cratio);

	float xd = sqrt((x0 - x1)*(x0 - x1) + (y0 - y1)*(y0 - y1) + (z0 - z1)*(z0 - z1)); //点间弦长。
	float an = asin(xd / 2); //弦长对应的半角，因为是弧度，所以就是对应的弧长。
	dist = 2 * an * ER; //全弧长，米。

	//北极坐标（0，0，1）
	//终点坐标（x1,y1,z1)

	//以上两个点投影到法平面上，然后计算他们连接起点的夹角。
	//法平面是连接地心和当前点的连线的垂面，即当前点的水平面
	//平面的法向量就是(x0,y0,z0),因为从球心射向这个点。
	//因此平面方程是 x0(x-x0)+y0(y-y0)+z0(z-z0)=0
	//把以上两个点都朝这个平面投影得到两个投影点。

	//对北极投影
	float d0 = z0 - 1.0;  //北极点到平面的距离。

	//北极点的投影坐标点 减去出发点 得到的是向量
	float cast_v0_x = 0 - x0 * d0 - x0;
	float cast_v0_y = 0 - y0 * d0 - y0;
	float cast_v0_z = 1 - z0 * d0 - z0;

	//对终点做投影坐标点 减去出发点 得到的是向量
	d0 = x1 * x0 + y1 * y0 + z1 * z0 - 1.0; //点到平面的距离。
	float cast_v1_x = x1 - x0 * d0 - x0;
	float cast_v1_y = y1 - y0 * d0 - y0;
	float cast_v1_z = z1 - z0 * d0 - z0;

	//向量夹角是 cos(an)= (a dot b)/mod(a)/mod(b)
	//求投影后的向量夹角。
	float dot = cast_v0_x * cast_v1_x + cast_v0_y * cast_v1_y + cast_v0_z * cast_v1_z;
	float ma = sqrt(cast_v0_x*cast_v0_x + cast_v0_y * cast_v0_y + cast_v0_z * cast_v0_z);
	float mb = sqrt(cast_v1_x*cast_v1_x + cast_v1_y * cast_v1_y + cast_v1_z * cast_v1_z);
	float cosx = dot / ma / mb; //夹角，0-180
	if (std::isnan(cosx) || std::isinf(cosx)) cosx = 0; //两个完全相同的坐标送进来时，mb=0,得到nan, 20231030补充。

	//计算误差可能导致超限
	if (cosx < -1.0) {
		angle = 180.0;
		return;
	}
	else if (cosx > 1.0) {
		angle = 0.0;
		return;
	}

	angle = acos(cosx)*cratio; //换为度数。

	//叉乘判断是否需要调节，仅需z分量
	float cz = -x0 * (y1 - y0) - (-y0)*(x1 - x0);
	if (cz > 0) angle = 360 - angle;

#undef ER
#undef cratio
}

//给出基点坐标，距离和角度，计算目标点的坐标值。20240320完善。
//现在，东经范围（0，180），西经（0，-180），北纬（0，90）,南纬（0，-90）
void global_dist_angle_to_gps(double lati0, double logi0, float dist, float angle, double& lati1, double& logi1)
{
#define ER 6380000
#define cratio  57.29578
	//连接本点和球心，根据方向找到对应的大圆的法线，距离换算为弧度，本点在大圆上移动这个弧度就对应了目标点的坐标值。
	//大圆的法线垂直于点到球心的连线，还垂直于方向角，这样可以求出大圆的法线。

	//本地三维坐标值。
	double r0 = cos(lati0 / cratio);
	double x0 = r0 * cos(logi0 / cratio);
	double y0 = r0 * sin(logi0 / cratio);
	double z0 = sin(lati0 / cratio);


	//本点指向北极的向量
	double nx0 = 0 - x0;
	double ny0 = 0 - y0;
	double nz0 = 1 - z0;


	//本点指向地心的向量
	double cx0 = 0 - x0;
	double cy0 = 0 - y0;
	double cz0 = 0 - z0;

	//本点，北极，地心三者构成的平面的法向量，用指向北极的向量叉乘指向地心的向量
	//两个单位向量的叉乘不一定是单位向量。
	double ai = ny0 * cz0 - nz0 * cy0;
	double aj = nz0 * cx0 - nx0 * cz0;
	double ak = nx0 * cy0 - ny0 * cx0;


	//此平面法线绕着 本点指向地心的向量旋转angle角度，即得到运动曲线对应的大圆的法线方向。
	//向量绕向量旋转的公式为 
	//v' = vcosθ + (u×v)sinθ + (u·v)u(1-cosθ)   v是旋转向量，u为旋转轴，都是3维的。
	//这个公式是网上找的。
	//u=(cx0,cy0,cz0) v=(ai,aj,ak) 
	double uxv_i = cy0 * ak - cz0 * aj;
	double uxv_j = cz0 * ai - cx0 * ak;
	double uxv_k = cx0 * aj - cy0 * ai;
	double udotv = cx0 * ai + cy0 * aj + cz0 * ak;
	double cos_angle = cos(angle / cratio);
	double sin_angle = sin(angle / cratio);

	//轨迹线所在的大圆平面对应的法线，这个法线是旋转轴，似乎必须单位化，否则套用公式会有误差。
	double bi = ai * cos_angle + uxv_i * sin_angle + udotv * (1 - cos_angle)*cx0;
	double bj = aj * cos_angle + uxv_j * sin_angle + udotv * (1 - cos_angle)*cy0;
	double bk = ak * cos_angle + uxv_k * sin_angle + udotv * (1 - cos_angle)*cz0;

	double mod_ch4 = sqrt(bi*bi + bj * bj + bk * bk);
	bi /= mod_ch4;
	bj /= mod_ch4;
	bk /= mod_ch4;

	//根据行进的距离，可以找到在大圆上运行的角度。
	double move_angle = dist / ER; //弧度。

	//当前点连接球心的向量，绕着轨迹线所在的大圆平面的法线，旋转move_angle角度，即得到了球心指向目标点的向量。
	//这同样是一个向量绕另一个向量的旋转。
	//这回u=(bi,bj,bk) v=(x0,y0,z0) 后者采用地心指向本点的向量，最后算出来的就是地心指向目标点的向量。
	uxv_i = bj * z0 - bk * y0;
	uxv_j = bk * x0 - bi * z0;
	uxv_k = bi * y0 - bj * x0;

	udotv = bi * x0 + bj * y0 + bk * z0;
	cos_angle = cos(move_angle);
	sin_angle = sin(move_angle);

	//目标点
	double ti = x0 * cos_angle + uxv_i * sin_angle + udotv * (1 - cos_angle)*bi;
	double tj = y0 * cos_angle + uxv_j * sin_angle + udotv * (1 - cos_angle)*bj;
	double tk = z0 * cos_angle + uxv_k * sin_angle + udotv * (1 - cos_angle)*bk;

	double mod = sqrt(ti*ti + tj * tj + tk * tk);
	ti /= mod;
	tj /= mod;
	tk /= mod;
	//最后得到的旋转向量的三个数, 即对应目标点的坐标值
	//将坐标值换算为经纬度。
	lati1 = asin(tk)*cratio; //北球正，南球负
	if (tj >= 0) {
		//东球。
		logi1 = atan2(tj, ti)*cratio;
	}
	else {
		//西球
		logi1 = atan2(tj, ti)*cratio; //得到的是顺时针从x轴旋转过去的角度负值， 0- -180度。西京90度为-90.
		//logi1 += 360; //转换后，西京90度变为 270度。
	}

#undef ER
#undef cratio
}

// 计算两个角度之间的最小差值（考虑-180~180循环）
float angularDistance(float a, float b) {
    float diff = fabs(a - b);
    return std::min(diff, 360.0f - diff);
}

// 计算角度的平均值 -180~180
float circularMean(const std::vector<float>& angles) 
{
    float sum_sin = 0.0f, sum_cos = 0.0f;
    
    for (float angle : angles) {
        float rad = angle  / 57.2958f;
        sum_sin += sin(rad);
        sum_cos += cos(rad);
    }
    
   
    float mean_deg = atan2(sum_sin, sum_cos)*57.2958;
    return mean_deg;
    // 规范化到[-180, 180)范围
    //mean_deg = fmod(mean_deg + 180.0f, 360.0f);
    //return (mean_deg >= 0) ? mean_deg - 180.0f : mean_deg + 180.0f;
}