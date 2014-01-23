#ifndef CATEGORIZER_ICE
#define CATEGORIZER_ICE

module RoboComp
{

    sequence<byte> CategorizerBufferedData;
    
    class CategorizerData 
    {
        /* Component callback data here */
    };

    interface CategorizerConsumer
    {
        idempotent void setData( CategorizerData data );
        nonmutating string getObjectUUID();
    };

    interface Categorizer
    {
        nonmutating void readCategorizerBufferedData( out CategorizerBufferedData gbd );
        idempotent void writeCategorizerBufferedData( CategorizerBufferedData gbd );
        bool subscribe( CategorizerConsumer *suscriber ) /*throws exception ...*/;
        idempotent bool unsubscribe( CategorizerConsumer *suscriber );
        nonmutating CategorizerData getData();
        nonmutating string getObjectUUID();
        

        // Component remote methods here ...
    };
};

#endif
