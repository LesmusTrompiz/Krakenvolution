
struct CinematicParams								//Estructura para los parametros mecanicos (motor + controladora)
{
    public:
	    double aceleracion;				//Aceleraci�n de la controladora de 3000 rpm/s
		double deceleracion;			//Valor de deceleracion de los motores
		double reductora;				//Reduccion de velocidad de 26
		double vel_eje_max;				//Velocidad m�xima del eje del motor segun datasheet
		double vel_max;					//Velocidad m�xima de la rueda
		double pulsos_por_rev;			//Pulsos por revolucion del encoder
		double L;						//Longitud del eje
		double diametro;				//Diametro de la rueda
        CinematicParams(
            double aceleracion_,	    //Aceleraci�n de la controladora de 3000 rpm/s
            double deceleracion_,	    //Valor de deceleracion de los motores
            double reductora_,		    //Reduccion de velocidad de 26
            double vel_eje_max_,	    //Velocidad m�xima del eje del motor segun datasheet
            double vel_max_,		    //Velocidad m�xima de la rueda
            double pulsos_por_rev_,	    //Pulsos por revolucion del encoder
            double L_,				    //Longitud del eje
            double diametro_);		    //Diametro de la rueda)
};

CinematicParams::CinematicParams(
        double aceleracion_,	    
        double deceleracion_,	    
        double reductora_,		    
        double vel_eje_max_,	    
        double vel_max_,		    
        double pulsos_por_rev_,	    
        double L_,				    
        double diametro_)		    
    :
        aceleracion{aceleracion_},
		deceleracion{deceleracion_},
		reductora{reductora_},
		vel_eje_max{vel_eje_max_},
		vel_max{vel_eje_max_},
		pulsos_por_rev{pulsos_por_rev_},
		L{L_},
		diametro{diametro_} 
        {};



struct ConsignaMotor								//Estructura para los parametros mecanicos (motor + controladora)
{
    public:
	    double aceleracion;				//Aceleraci�n de la controladora de 3000 rpm/s
		double deceleracion;			//Valor de deceleracion de los motores
		double reductora;				//Reduccion de velocidad de 26
		double vel_eje_max;				//Velocidad m�xima del eje del motor segun datasheet
		double vel_max;					//Velocidad m�xima de la rueda
		double pulsos_por_rev;			//Pulsos por revolucion del encoder
		double L;						//Longitud del eje
		double diametro;				//Diametro de la rueda
        ConsignaMotor(
            double aceleracion_,	    //Aceleraci�n de la controladora de 3000 rpm/s
            double deceleracion_,	    //Valor de deceleracion de los motores
            double reductora_,		    //Reduccion de velocidad de 26
            double vel_eje_max_,	    //Velocidad m�xima del eje del motor segun datasheet
            double vel_max_,		    //Velocidad m�xima de la rueda
            double pulsos_por_rev_,	    //Pulsos por revolucion del encoder
            double L_,				    //Longitud del eje
            double diametro_);		    //Diametro de la rueda)
};

struct ErrorMotor								//Estructura para los parametros mecanicos (motor + controladora)
{
    public:
	    double aceleracion;				//Aceleraci�n de la controladora de 3000 rpm/s
		double deceleracion;			//Valor de deceleracion de los motores
		double reductora;				//Reduccion de velocidad de 26
		double vel_eje_max;				//Velocidad m�xima del eje del motor segun datasheet
		double vel_max;					//Velocidad m�xima de la rueda
		double pulsos_por_rev;			//Pulsos por revolucion del encoder
		double L;						//Longitud del eje
		double diametro;				//Diametro de la rueda
        ErrorMotor(
            double aceleracion_,	    //Aceleraci�n de la controladora de 3000 rpm/s
            double deceleracion_,	    //Valor de deceleracion de los motores
            double reductora_,		    //Reduccion de velocidad de 26
            double vel_eje_max_,	    //Velocidad m�xima del eje del motor segun datasheet
            double vel_max_,		    //Velocidad m�xima de la rueda
            double pulsos_por_rev_,	    //Pulsos por revolucion del encoder
            double L_,				    //Longitud del eje
            double diametro_);		    //Diametro de la rueda)
};