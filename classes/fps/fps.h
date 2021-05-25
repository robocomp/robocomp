class FPSCounter
{
	public:
		FPSCounter()
		{
			begin = std::chrono::high_resolution_clock::now();
		}
    float print( const std::string &text, const unsigned int msPeriod = 1000)
		{
			auto end = std::chrono::high_resolution_clock::now();
			auto elapsed = std::chrono::duration<double>(end - begin).count() * 1000;
			if( elapsed > msPeriod)
			{
				std::cout << "Epoch time = " << elapsed/cont << "ms. Fps = " << cont << " " << text << std::endl;
				begin = std::chrono::high_resolution_clock::now();
				cont = 0;
			}
			cont++;
			return count;
		}
		std::chrono::time_point<std::chrono::high_resolution_clock> begin;
		std::uint32_t cont = 0;
};

