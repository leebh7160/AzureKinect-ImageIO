#include "ImageReader.h"

using namespace std;

SensorData::SensorData() {}
SensorData::SensorData(
	const cv::Mat & rgb,
	const cv::Mat & depth,
	int id,
	double stamp) : SeqID(id), _rgb(rgb), _depth(depth)
{
}

stringmat read_directory(const std::string& name, stringmat v)
{
	string pattern(name);
	pattern.append("\\*");
	WIN32_FIND_DATA data;
	HANDLE hFind;

	if ((hFind = FindFirstFile(pattern.c_str(), &data)) != INVALID_HANDLE_VALUE)
	{
		do {
			v.push_back(data.cFileName);
		} while (FindNextFile(hFind, &data) != 0);

		FindClose(hFind);
	}
	return v;
}

SensorData ImageReader::ImageRead()
{
	SensorData data;

	cv::Mat instante;
	//다량의 이미지 담기(스레드로 변경 가능할 듯)
	for (int j = 0; j < v.size() / 2; j++)
	{
		if (j < v.size())
		{
			stringstream jj;
			jj << j;

			//컬러 이미지, 깊이 이미지 찾기
			colorpath = "C:\\Users\\User\\Desktop\\test\\color" + jj.str() + ".jpg";
			depthpath = "C:\\Users\\User\\Desktop\\test\\test" + jj.str() + ".jpg";

			instante = cv::imread(colorpath);
			if (instante.data != NULL)
			{
				cv::imshow("dd", instante);
				bgrCV = instante;
			}
			instante.release();

			instante = cv::imread(depthpath);
			if (instante.data != NULL)
			{
				cv::imshow("ff", instante);
				depthCV = instante;
			}

			//컬러 이미지, 깊이 이미지 담기
			instante.release();

			if (cv::waitKey(1) >= 0)
			{
				break;
			}

			data = SensorData(bgrCV, depthCV, j, 0);
			cout << data.SeqID << endl;
		}
	}
	return data;

}

void ImageReader::init()
{
	num << i;

	//폴더 내부 파일 개수 확인
	v = read_directory("C:\\Users\\User\\Desktop\\test", v);

	std::copy(v.begin(), v.end(),
		std::ostream_iterator<std::string>(std::cout, "\n"));
	cout << v.size() / 2 << endl;

	
	//ImageRead();

	cout << ImageRead().SeqID << endl;


}
