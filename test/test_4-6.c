#include "spdk/stdinc.h"
#include "spdk/ioat.h"
#include "spdk/env.h"
#include "spdk/queue.h"
#include "spdk/string.h"
#include "spdk/copy_engine.h"
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#define ELE_NUM 8
struct channel
{
	struct spdk_ioat_chan *chan;
	TAILQ_ENTRY(channel) tailq;
};
struct core
{
	int core_id;
	TAILQ_HEAD(, channel) attached_chan;
};
struct element
{
	uint64_t size;
};
// static TAILQ_HEAD(,channel) user_channel;
struct worker
{
	uint64_t *src;
	uint64_t *dst;
	uint64_t size;
	struct spdk_ioat_chan *chan;
	uint64_t core;
	bool flag;
};
struct user_config
{
	int size;
	int core_num;
	char *core_mask;
	TAILQ_HEAD(, channel) user_channel;
	TAILQ_HEAD(, core) user_core;
	int chan_num;
	int chan_id;
} user_config;
static bool
probe_cb(void *cb_ctx, struct spdk_pci_device *pci_dev)
{
	// printf("Found matching device at %04x:%02x:%02x.%x,vendor:0x%04x,device:0x%04x,socket:%d\n",
	//        spdk_pci_device_get_domain(pci_dev),
	//        spdk_pci_device_get_bus(pci_dev), spdk_pci_device_get_dev(pci_dev),
	//        spdk_pci_device_get_func(pci_dev),
	//        spdk_pci_device_get_vendor_id(pci_dev), spdk_pci_device_get_device_id(pci_dev),
	// 	pci_dev->socket_id);
	return true;
}
int count = 0;
int g_channel = 0;
int n=0;
static void
attach_cb(void *cb_ctx, struct spdk_pci_device *pci_dev, struct spdk_ioat_chan *chan)
{
	n++;
	if(n>3&&n<5)
	{// else printf("%d \n",n);
	struct channel *temp;
	if (g_channel < user_config.chan_num)
	{
		temp = spdk_dma_zmalloc(sizeof(*temp), 0, NULL);
		if (temp == NULL)
		{
			printf("Failed to allocate device struct\n");
			return;
		}
		temp->chan = chan;
		
		TAILQ_INSERT_TAIL(&user_config.user_channel, temp, tailq);
		printf("user get device at %04x:%02x:%02x.%x,vendor:0x%04x,device:0x%04x,socket:%d\n",
	       spdk_pci_device_get_domain(pci_dev),
	       spdk_pci_device_get_bus(pci_dev), spdk_pci_device_get_dev(pci_dev),
	       spdk_pci_device_get_func(pci_dev),
	       spdk_pci_device_get_vendor_id(pci_dev), spdk_pci_device_get_device_id(pci_dev),pci_dev->socket_id);

		g_channel++;
	}
	}

}
struct result
{
	uint64_t time;
	double speed;
};
void display_header()
{
	printf("\n");
	printf("|---------------|-------------|---------------|--------------|--------------|--------------|--------------|\n");
	printf("| Copy          | Memcpy                      | SPDK (one channel)          | SPDK (two channel)          |\n");
	printf("| size          | Time        |   Speed       | Time         |Speed         | Time         |Speed         |\n");
	printf("|---------------|-------------|---------------|--------------|--------------|--------------|--------------|\n");
}
static void ioat_done(void *arg)
{
	struct worker *temp = (struct worker *)arg;

	// if(memcmp((void *)temp->src, (void *)temp->dst, temp->size)==0)
	// {
	temp->flag = true;
	count++;
	// printf("success!%d\n",count);
	// 	printf("%lx %lx\n",*(uint64_t *)(temp->src),*(uint64_t *)(temp->dst));
	// }
	// else printf("fail \n");
}
#define TIME 100000000
#define TIME_SEC 1
#define NUMA_NUM 4
#define LOCAL_NUMA_NUM 0
#define REMOTE_LOCAL_NUMA_NUM 1
#define SIZE pow(2,30)*1
static int
opts_init_env(void)
{
	struct spdk_env_opts opts;
	spdk_env_opts_init(&opts);
	opts.name = "test";
	opts.core_mask = user_config.core_mask;
	if (spdk_env_init(&opts) < 0)
		return -1;
	// printf("1\n");
	TAILQ_INIT(&user_config.user_channel);
	TAILQ_INIT(&user_config.user_core);
	if (spdk_ioat_probe(NULL, probe_cb, attach_cb) != 0)
	{
		printf(stderr, "ioat_probe() failed\n");
		return -1;
	}
	// printf("1\n");
	channel_attach_core();
	return 0;
}
void work_fn(struct worker *worker)
{
	// printf("core %d channel is begin!",worker->core);
	// printf("\n");
	// tsc_end = spdk_get_ticks() + TIME_SEC * spdk_get_ticks_hz();
	if (!worker->chan)
	{
		printf("work_fn:core %d channel is NULL!", worker->core);
		exit(0);
	}
	// else printf("worker is startint on core%d!\n",worker->core);

	uint64_t i, n = 0;
	struct result res;
	// struct timespec start, end;
	// uint64_t timeuse=0;
	count = 0;
	
	spdk_ioat_submit_copy(
			worker->chan,
			worker,
			ioat_done,
			worker->dst,
			worker->src,
			worker->size);
	// do
	// {
	//   spdk_ioat_process_events(worker->chan);
	// 	// printf("polling\n");
	// } while (!worker->flag);
	// clock_gettime(CLOCK_REALTIME, &start);
	// clock_gettime(CLOCK_REALTIME, &end);
	// n++;
	// spdk_free(src);
	// spdk_free(dst);
	// timeuse=timeuse+(end.tv_sec*pow(10,9)+end.tv_nsec)-(start.tv_sec*pow(10,9)+start.tv_nsec);
	// res.time=timeuse/n;
	// res.speed=((double)worker->size*n/(1024*1024))/((double)timeuse/pow(10,9));
	// count=0;
	// printf("%2d       %12uns| %8.3lfMiB/s|           |\n",worker->core,res.time,res.speed);
}
struct result seq_memcpy(struct worker *worker)
{
	memcpy(worker->dst,worker->src,worker->size);
}

parse_args(int argc, char **argv)
{
	int op;
	while ((op = getopt(argc, argv, "c:hn:o:q:t:v")) != -1)
	{
		switch (op)
		{
		case 'o':
			user_config.size = spdk_strtol(optarg, 10);
			break;
		case 'n':
			user_config.chan_num = spdk_strtol(optarg, 10);
			break;
		case 'q':
			// user_config.queue_depth = spdk_strtol(optarg, 10);
			break;
		case 't':
			// user_config.time_in_sec = spdk_strtol(optarg, 10);
			break;
		case 'c':
			user_config.core_mask = optarg;
			break;
		case 'v':
			// user_config.verify = true;
			break;
		case 'chan_id':
			user_config.chan_id= spdk_strtol(optarg, 10);;

					}
	}
	

	return 0;
}
void channel_attach_core()
{
	int k;
	// struct core *core=TAILQ_FIRST(&user_config.user_core);
	struct core *core = spdk_dma_zmalloc(sizeof(*core), 0, NULL);
	core->core_id=spdk_env_get_current_core();  
	struct channel *temp ;
	// do
	// {	
	// 	SPDK_ENV_FOREACH_CORE(k)
	// 	{
	// 		core = spdk_dma_zmalloc(sizeof(*core), 0, NULL);
	// 		core->core_id = k;
	// 		TAILQ_INIT(&core->attached_chan);
	// 		TAILQ_INSERT_TAIL(&core->attached_chan, temp, tailq);
	// 		printf("core %d attached channel %lu!\n",core->core_id,temp->chan);
	// 		temp=TAILQ_NEXT(temp,tailq);
	// 	}

	// }while(!temp);
	TAILQ_INIT(&core->attached_chan);
	printf("here\n");
	TAILQ_FOREACH(temp,&user_config.user_channel, tailq)
	{
		// TAILQ_INSERT_TAIL(&core->attached_chan, temp, tailq);
		// printf("core %d attached channel 0x%lx!\n",core->core_id,temp->chan);
	}
}

int main(int argc, char **argv)
{
	parse_args(argc, argv);
	if (opts_init_env() < 0)
	{
		printf("opts init environment fail.\n");
		return 0;
	}
	else
		printf("opts init environment success.\n");

	struct element e[ELE_NUM];
	for (int i = 0; i < ELE_NUM; i++)
	{
		e[i].size = pow(2, 25) * (1 + i);
	}
	struct timespec start, end;
	struct result temp;
	struct channel *channel;
	struct worker worker[16];
	uint64_t master_core = spdk_env_get_current_core(),timeuse;
	printf("master_core is %d in socket %d\n",master_core,spdk_env_get_socket_id(master_core));
	int i=0;
	TAILQ_FOREACH(channel, &user_config.user_channel, tailq)
	{
		// uint64_t *src = spdk_malloc(SIZE, 0, NULL, (spdk_env_get_socket_id(master_core)+REMOTE_LOCAL_NUMA_NUM)%NUMA_NUM, SPDK_MALLOC_DMA);
		worker[i].src=spdk_malloc(SIZE, 0, NULL, (spdk_env_get_socket_id(master_core)+LOCAL_NUMA_NUM)%NUMA_NUM, SPDK_MALLOC_DMA);
		worker[i].dst=spdk_malloc(SIZE, 0, NULL, (spdk_env_get_socket_id(master_core)+REMOTE_LOCAL_NUMA_NUM)%NUMA_NUM, SPDK_MALLOC_DMA);
		
		if (!worker[i].src || !worker[i].dst)
		{
			printf("spdk_malloc failed!\n");
			return 0;
		}
		else
		{
			// printf("worker[%d].src==0x%x,worker[%d].dst=0x%x\n",i,worker[i].src,i,worker[i].dst);
		}
		
		memset(worker[i].src,0,SIZE);
		memset(worker[i].dst,1,SIZE);
		worker[i].size=SIZE;
		worker[i].chan=channel->chan;
		printf("attached channel 0x%lx\n",worker[i].chan);
		work_fn(&worker[i]);
		i++;
		
	}

	clock_t start_t, end_t;
	start_t = clock();
	clock_gettime(CLOCK_REALTIME, &start);
	

	// clock_gettime(CLOCK_REALTIME, &end);
	// end_t = clock();
	// timeuse=(end.tv_sec*pow(10,9)+end.tv_nsec)-(start.tv_sec*pow(10,9)+start.tv_nsec);
	// printf("I/OAT timeuse=%luns\n",timeuse);
	// double total_t = (double)(end_t - start_t) / CLOCKS_PER_SEC;
    // printf("I/OAT 占用的总时间：%lf\n", total_t  );


	// start_t = clock();
	// clock_gettime(CLOCK_REALTIME, &start);
	// for(i=0;i<user_config.chan_num;i++)
	// {
	// 		seq_memcpy(&worker[i]);
	// }
	// clock_gettime(CLOCK_REALTIME, &end);
	// end_t = clock();
	// timeuse=(end.tv_sec*pow(10,9)+end.tv_nsec)-(start.tv_sec*pow(10,9)+start.tv_nsec);
	// printf("memcpy timeuse=%luns\n",timeuse);
	// double total_t1 = (double)(end_t - start_t) / CLOCKS_PER_SEC;
    // printf("memcpy 占用的总时间：%lf\n", total_t1  );

	// sleep(1000);
	
	do{
	TAILQ_FOREACH(channel, &user_config.user_channel, tailq)
	{
		spdk_ioat_process_events(channel->chan);
		// printf("%d\n",count);
	}
	}while (count!=user_config.chan_num);
	

	clock_gettime(CLOCK_REALTIME, &end);
	timeuse=(end.tv_sec*pow(10,9)+end.tv_nsec)-(start.tv_sec*pow(10,9)+start.tv_nsec);
	uint64_t speed=((double)worker->size*count/(1024*1024))/((double)timeuse/pow(10,9));
	printf("%luns,%luMB/s\n",timeuse,speed);
	display_header();
	int k = 0;
	bool flag[64];
}
