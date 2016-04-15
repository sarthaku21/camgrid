class settings{
	public:
		float wb;
		bool auto_wb;
		float exposure;
		bool auto_exposure;
		float gain;
		bool auto_gain;
		
		settings(){
			auto_wb = true;
			wb = 0;
			auto_exposure = true;
			exposure = 0;
			auto_exposure = false;
			gain = 0;
			auto_gain = false;
		}
		
		void loadStettings(float* vals){
			wb = vals[0];
			if(wb < 998)
				auto_wb = false;
			exposure = vals[1];
			if(exposure < 998)
				auto_exposure = false;
			gain = vals[2];
			if(gain < 998)
				auto_gain = false;
		}
	};
