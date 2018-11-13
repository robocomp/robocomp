/*
 * Copyright 2018 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef HETEROCONTAINER_H
#define HETEROCONTAINER_H

#include <vector>
#include <unordered_map>
#include <functional>
#include <iostream>
#include <experimental/type_traits>

namespace andyg
{
	
	template<class...>
	struct type_list{};
		
	template<class... TYPES>
	struct visitor_base
	{
			using types = andyg::type_list<TYPES...>;
	};
      
	class HeteroContainer
	{
		public:
			HeteroContainer() = default;
			HeteroContainer(const HeteroContainer& _other)
			{
					*this = _other;
			}
			
			HeteroContainer& operator=(const HeteroContainer& _other)
			{
					clear();
					clear_functions = _other.clear_functions;
					copy_functions = _other.copy_functions;
					size_functions = _other.size_functions;
					for (auto&& copy_function : copy_functions)
					{
							copy_function(_other, *this);
					}
					return *this;
			}
			
			template<class T>
			using MapType = std::unordered_map<const HeteroContainer*, std::unordered_map<std::uint32_t,T>>;
	
// 			template<class T>
// 			void push_back(const T& _t)
// 			{
// 					// don't have it yet, so create functions for printing, copying, moving, and destroying
// 					if (items<T>.find(this) == std::end(items<T>))
// 					{   
// 							clear_functions.emplace_back([](HeteroContainer& _c){items<T>.erase(&_c);});
// 							
// 							// if someone copies me, they need to call each copy_function and pass themself
// 							copy_functions.emplace_back([](const HeteroContainer& _from, HeteroContainer& _to)
// 																					{
// 																							items<T>[&_to] = items<T>[&_from];
// 																					});
// 							size_functions.emplace_back([](const HeteroContainer& _c){return items<T>[&_c].size();});
// 							//search_functions.emplace_back([](const HeteroContainer& _c){return items<T>[this].at(_t);});
// 					}
// 					items<T>[this].push_back(_t);
// 			}
			
			template<class T>
			void insert(const std::uint32_t &key, const T& _t)
			{
					// don't have it yet, so create functions for printing, copying, moving, and destroying
					if (items<T>.find(this) == std::end(items<T>))
					{   
							clear_functions.emplace_back([](HeteroContainer& _c){items<T>.erase(&_c);});
							
							// if someone copies me, they need to call each copy_function and pass themself
							copy_functions.emplace_back([](const HeteroContainer& _from, HeteroContainer& _to)
																					{
																							items<T>[&_to] = items<T>[&_from];
																					});
							size_functions.emplace_back([](const HeteroContainer& _c){ return items<T>[&_c].size();});
					}
					items<T>[this].insert(std::make_pair(key,_t));
					
			}
			
			
			void clear()
			{
					for (auto&& clear_func : clear_functions)
					{
							clear_func(*this);
					}
			}
			
			template<class T>
			size_t number_of() const
			{
					auto iter = items<T>.find(this);
					if (iter != items<T>.cend())
							return items<T>[this].size();
					return 0;
			}
			
			size_t size() const
			{
					size_t sum = 0;
					for (auto&& size_func : size_functions)
					{
							sum += size_func(*this);
					}
					return sum;
			}
		
		  // throws a std::out_of_range exception if key is not found
		  template<class T>
			auto at(const std::uint32_t &key) const
			{
					auto iter = items<T>.find(this);
					if (iter != items<T>.cend())
						return items<T>[this].at(key);
					//return T();
			}
			
			template<class T>
			auto find(const std::uint32_t &key) const
			{
					auto iter = items<T>.find(this);
					if (iter != items<T>.cend())
						return items<T>[this].find(key);
					//return items<T>[this].cend();
			}
			
			template<class T>
			auto erase(const std::uint32_t &key) 
			{
					auto iter = items<T>.find(this);
					if (iter != items<T>.cend())
						return items<T>[this].erase(key);
			}
			
			template<class T>
			auto getMap() const
			{
					auto iter = items<T>.find(this);
					if (iter != items<T>.cend())
						return items<T>[this];
					//return MapType<T>;
			}
			
			template<class T>
			auto count(const std::uint32_t &key) const
			{
					auto iter = items<T>.find(this);
					if (iter != items<T>.cend())
						return items<T>[this].count(key);
			}
			
			~HeteroContainer()
			{
					clear();
			}   
			
			template<class T>
			void visit(T&& visitor)
			{
					visit_impl(visitor, typename std::decay_t<T>::types{});
			}
			
			private:
				template<class T>
				static MapType<T> items;
				//static std::unordered_map<const HeteroContainer*, std::vector<T>> items;
				//static std::unordered_map<const HeteroContainer*, std::unordered_map<std::uint32_t,T>> items;
				
				template<class T, class U>
				using visit_function = decltype(std::declval<T>().operator()(std::declval<U&>()));
				template<class T, class U>
				static constexpr bool has_visit_v = std::experimental::is_detected<visit_function, T, U>::value;
					
				template<class T, template<class...> class TLIST, class... TYPES>
				void visit_impl(T&& visitor, TLIST<TYPES...>)
				{
						(..., visit_impl_help<std::decay_t<T>, TYPES>(visitor));
				}
				
				template<class T, class U>
				void visit_impl_help(T& visitor)
				{
						static_assert(has_visit_v<T, U>, "Visitors must provide a visit function accepting a reference for each type");
						//for (auto&& element : items<U>[this])
						for (auto& [ key, element ]	: items<U>[this])
								visitor(element);
				}
				
				std::vector<std::function<void(HeteroContainer&)>> clear_functions;
				std::vector<std::function<void(const HeteroContainer&, HeteroContainer&)>> copy_functions;
				std::vector<std::function<size_t(const HeteroContainer&)>> size_functions;
	};

	//template<class T>
	//std::unordered_map<const HeteroContainer*, std::vector<T>> HeteroContainer::items;
	template<class T>
	std::unordered_map<const HeteroContainer*, std::unordered_map<std::uint32_t, T>> HeteroContainer::items;
	
}

#endif // HETEROCONTAINER_H
