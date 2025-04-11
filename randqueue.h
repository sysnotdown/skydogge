
#ifndef _RANDQUEUE_H
#define _RANDQUEUE_H

#include <stdio.h>

#include <list>
#include <queue>
#include <vector>
template<typename _T>
class NRandQueue
{
public:
	NRandQueue()
	{
		_tail = 0;
		_data.resize(200);
	}

	NRandQueue(size_t capacity)
	{
		_tail = 0;
		_data.resize(capacity);
	}

	void resize(size_t capacity)
	{
		_tail=0;
		_data.resize(capacity);
	}

	size_t size() {
		return _data.size();
	}
	
	size_t cont_size() {
		return  _data.size();
	}

	void push(const _T& obj) {
		_data[_tail++] = obj;
		_tail %= _data.size();
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
		// if (n >= _data.size()) {
		// 	size_t m = n % _data.size();
		// 	if (_tail > m) return _data[_tail - m - 1];
		// 	else return _data[_data.size() + _tail - m -1];
		// }
		// else {
		// 	if (_tail > n) return _data[_tail - n -1];
		// 	else return _data[_data.size() + _tail - n -1];
		// }

		//上面代码简化为这个
		size_t m= n%_data.size();
		if(_tail > m) return _data[_tail -m -1];
		else return _data[_data.size() + _tail -m -1];

		// //上面的代码应该没有问题，下面的代码有问题
		// size_t tc= _tail;
		// while(tc<n) tc += _data.size();
		// return _data[tc - n]; //保证tc-n>=0 <_data.size();

		//这个代码才是等价的
		// size_t tc = _tail;
		// while (tc <= n) tc += _data.size();
		// return _data[tc - n-1];
	}

	const _T& last(size_t n=0) const
	{
		// if (n >= _data.size()) {
		// 	size_t m = n % _data.size();
		// 	if (_tail > m) return _data[_tail - m -1];
		// 	else return _data[_data.size() + _tail - m -1];
		// }
		// else {
		// 	if (_tail > n) return _data[_tail - n -1];
		// 	else return _data[_data.size() + _tail - n -1];
		// }

		//上面代码简化为这个
		size_t m= n%_data.size();
		if(_tail > m) return _data[_tail -m -1];
		else return _data[_data.size() + _tail -m -1];

		// //上面的代码应该没有问题，下面的代码有问题
		// size_t tc= _tail;
		// while(tc<n) tc += _data.size();
		// return _data[tc - n]; //保证tc-n>=0 <_data.size();

		//这个代码才是等价的
		// size_t tc = _tail;
		// while (tc <= n) tc += _data.size();
		// return _data[tc - n-1];
	}

	_T& front(size_t n=0)
	{

		size_t p = _tail + n;
		p  %= _data.size();
		return _data[p];

	}

	const _T& front(size_t n=0) const
	{

		size_t p = _tail + n;
		p  %= _data.size();
		return _data[p];

	}
private:
	std::vector<_T> _data;
	size_t _tail;
};

#endif