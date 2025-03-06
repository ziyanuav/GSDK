/************************************************************************
File name   : uthread.h
Description : Thread class
Author      : lijia
Date        : 2025/01/3
Version     : V1.0.0
History     : created by Lijia
************************************************************************/
#ifndef UTHREAD_H_
#define UTHREAD_H_

#include <pthread.h>

typedef enum EThreadType{
    PTHREAD_NORMAL,
    PTHREAD_DETACHED,
}ThreadType;

typedef struct ThreadContext{
    ThreadContext(){
        self = NULL;
        param = NULL;
    }

    void *self;
    void *param;
}*PThreadContext;


class UThread
{
public:
	UThread();
	virtual ~UThread();

    pthread_t Start(void *param, int threadType = PTHREAD_NORMAL,bool priority = false);
	void Wait(pthread_t thread);

private:
	static void* StartThread(void* ptr);
	virtual void HandleRun(void *param) = 0;
};


#endif /* UTHREAD_H_ */
