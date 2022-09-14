// Example: fps.print("FPS:");

class FPSCounter
{
	public:
		FPSCounter()
		{
			begin = std::chrono::high_resolution_clock::now();
		}
        int print( const std::string &text, const unsigned int msPeriod = 1000)
        {
            static int fps=0;
            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration<double>(end - begin).count() * 1000;
            if( elapsed > msPeriod)
            {
				last_period = elapsed/cont;
                std::cout << "Epoch time = " << last_period << "ms. Fps = " << cont << " " << text << std::endl;
                begin = std::chrono::high_resolution_clock::now();
                fps=cont;
                cont = 0;
            }
            cont++;
            return fps;
        }
		void print( const std::string &text, std::function<void(int)> f, const unsigned int msPeriod = 1000)
		{	
			auto end = std::chrono::high_resolution_clock::now();
			auto elapsed = std::chrono::duration<double>(end - begin).count() * 1000;
			if( elapsed > msPeriod)
			{
				last_period = elapsed/cont;
				std::cout << "Epoch time = " << last_period << "ms. Fps = " << cont << " " << text << std::endl;
				begin = std::chrono::high_resolution_clock::now();
				f(cont);
				cont = 0;
			}
			cont++;
		}
		float get_period() const {return last_period;}

		std::chrono::time_point<std::chrono::high_resolution_clock> begin;
		int cont = 0;
		float last_period = 0;
};

