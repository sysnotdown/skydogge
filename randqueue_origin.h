
#ifndef _RANDQUEUE_ORIGIN_H
#define _RANDQUEUE_ORIGIN_H

//原始版本，日志缓存需要。

#include <stdio.h>

#include <list>
#include <queue>
#include <vector>
template<typename _T>
class NRandQueue_Origin
{
public:
	NRandQueue_Origin()
	{
		_tail = 0;
		_num = 0;
		_data.resize(200);
	}

	NRandQueue_Origin(size_t capacity)
	{
		_tail = 0;
		_num = 0;
		_data.resize(capacity);
	}

	void resize(size_t capacity)
	{
		_tail=0;
		_num=0;
		_data.resize(capacity);
	}

	size_t size() {
		return _num;
	}
	
	size_t cont_size() {
		return (std::min)(_num, _data.size());
	}

	void push(const _T& obj) {
		_data[_tail++] = obj;
		_tail %= _data.size();
		_num++;
	}


	_T& operator [] (size_t k) {
		return front(k);
	}

	const _T& operator [] (size_t k) const {
		return front(k);
	}

	//要求所有用到的权重和为1.0，不然外部要调节
	_T recent_weighted_mean(size_t n, float weight[]) {
		//tail位置就是最后一个数。
		_T sum = 0.0f;

		for (size_t i = 0; i < n; i++)
		{
			size_t p;
			if (_tail < i + 1) p = _tail + _data.size() - i - 1;
			else p = _tail - i - 1;
			sum += _data[p] * weight[i];
		}
		return sum;
	} //最近几个按权重调节的均值

	_T recent_mean(size_t n) {
		return history_mean(0, n);
	} //最近几个按权重调节的均值

	_T history_mean(size_t dist, size_t n)
	{
		_T sum = 0.0f;

		//这两句防止调用写错出现意外。
		if (n == 0) n = 1;

		for (size_t i = dist; i < n + dist; i++)
		{
			const _T& one = last(i);
			sum += one;
		}
		return sum / n;

	}; //历史记录里的均值

	


	_T& last(size_t n=0)
	{
		if (n >= _data.size()) {
			size_t m = n % _data.size();
			if (_tail > m) return _data[_tail - m - 1];
			else return _data[_data.size() + _tail - m -1];
		}
		else {
			if (_tail > n) return _data[_tail - n -1];
			else return _data[_data.size() + _tail - n -1];
		}

	}

	const _T& last(size_t n=0) const
	{
		if (n >= _data.size()) {
			size_t m = n % _data.size();
			if (_tail > m) return _data[_tail - m -1];
			else return _data[_data.size() + _tail - m -1];
		}
		else {
			if (_tail > n) return _data[_tail - n -1];
			else return _data[_data.size() + _tail - n -1];
		}

	}

	_T& front(size_t n=0)
	{
		if (_num >= _data.size())
		{
			size_t p = _tail + n;
			p  %= _data.size();
			return _data[p];
		}
		else
		{
			//未填满，head=0;
			n %= _num;
			return _data[n];
		}
	}

	const _T& front(size_t n=0) const
	{
		if (_num >= _data.size())
		{
			size_t p = _tail + n;
			p  %= _data.size();
			return _data[p];
		}
		else
		{
			//未填满，head=0;
			n %= _num;
			return _data[n];
		}
	}
private:
	std::vector<_T> _data;
	size_t _tail;
	size_t _num;
};

#endif