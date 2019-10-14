#include "spdk/stdinc.h"
#include "spdk/ioat.h"
#include "spdk/env.h"
#include "spdk/queue.h"
#include "spdk/string.h"
#include "spdk/barrier.h"
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <emmintrin.h>

void parse_args(int argc, char **argv, struct spdk_env_opts *opts)
{
	int op;
	while ((op = getopt(argc, argv, "c:hn:o:q:t:v")) != -1)
	{
		switch (op)
		{
		// case 'o':
		// 	user_config.size = spdk_strtol(optarg, 10);
		// 	break;
		// case 'n':
		// 	user_config.chan_num = spdk_strtol(optarg, 10);
		// 	break;
		case 'q':
			// user_config.queue_depth = spdk_strtol(optarg, 10);
			break;
		case 't':
			// user_config.time_in_sec = spdk_strtol(optarg, 10);
			break;
		case 'c':
			opts->core_mask = optarg;
			break;
		case 'v':
			// user_config.verify = true;
			break;
			// case 'chan_id':
			// 	user_config.chan_id= spdk_strtol(optarg, 10);
		}
	}
	return 0;
}

struct worker
{
	uint64_t *src;
	uint64_t *dst;
	uint64_t size;

	int chan_num;
	int worker_count;
};
int count = 0;

static bool
probe_cb(void *cb_ctx, struct spdk_pci_device *pci_dev)
{
	return true;
}

static void
attach_cb(void *cb_ctx, struct spdk_pci_device *pci_dev, struct spdk_ioat_chan *chan)
{
	// printf("1\n");
	// struct ioat_chan *temp=malloc(sizeof(struct ioat_chan));
	// spdk_dma_zmalloc(sizeof(struct ioat_chan), 0, NULL);
	// temp->chan = chan;
	// TAILQ_INSERT_TAIL(&global_config.global_chan, temp, next);
	// printf("global attached 0x%lx!\n",temp);
	// printf("user get device at %04x:%02x:%02x.%x,vendor:0x%04x,device:0x%04x,socket:%d\n",
	//     spdk_pci_device_get_domain(pci_dev),
	//     spdk_pci_device_get_bus(pci_dev), spdk_pci_device_get_dev(pci_dev),
	//     spdk_pci_device_get_func(pci_dev),
	// 	spdk_pci_device_get_vendor_id(pci_dev), spdk_pci_device_get_device_id(pci_dev),spdk_pci_device_get_socket_id(pci_dev)
	// 	);
	// global_config.chan_num++;
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

#define TIME 100000000
#define TIME_SEC 1
#define NUMA_NUM 4
#define LOCAL_NUMA_NUM 0
#define REMOTE_LOCAL_NUMA_NUM 1
static int
opts_init_env(int argc, char **argv)
{
	struct spdk_env_opts *opts = malloc(sizeof(struct spdk_env_opts));
	spdk_env_opts_init(opts);
	parse_args(argc, argv, opts);
	opts->name = "test";
	// opts->core_mask="0x1";
	if (spdk_env_init(opts) < 0)
		return -1;

	if (spdk_ioat_probe(NULL, probe_cb, attach_cb) != 0)
	{
		printf(stderr, "ioat_probe() failed\n");
		return -1;
	}
	// printf("here\n");
	return 0;
}
#define N 10000
int work_fn()
{	
	int a[N];

	uint64_t size = pow(2, 31) ;
	uint64_t *src = spdk_malloc(size, pow(2, 21), NULL, spdk_env_get_current_core(), SPDK_MALLOC_DMA);
	uint64_t *dst = spdk_malloc(size, pow(2, 21), NULL, spdk_env_get_current_core(), SPDK_MALLOC_DMA);
	// uint64_t *src = malloc(size);
	// uint64_t *dst = malloc(size);
	if (!src || !dst)
	{
		printf("worker's src or dst malloc failed!\n");
		exit(0);
	}
	memset(src, 1, size);
	memset(dst, 2, size);
	if(memcmp(dst, src, size) == 0)
	{
		printf("src and dst are same\n");
		exit(0);
	}
	for (int i = 0; i < N; i++)
	{
		
		// printf("&temp = %x\n",&temp);
		// spdk_mb();
		int num = ioat_copy(dst, src, size, &a[i]);
		// ioat_copy_raw(dst, src, size, &a[i]);
		// while(memcmp(dst, src, size) != 0)



		
		while( a[i] < num )
		{
			// if(memcmp(dst, src, size) == 0)break;
			_mm_pause();
			// printf("pause! %d\n",spdk_env_get_current_core());	
		}
		// if( i == N-1 )
		// 	while( a[i] < 16 )
		// 	{
		// 		sleep(5);
		// 		_mm_pause();
		// 		printf("core %d is pausing!\n",spdk_env_get_current_core());
		// 		for (int i = 0; i < N; i++)
		// 		{
		// 			printf("core %d:a[%d] = %d\n", spdk_env_get_current_core(), i, a[i]);
		// 		}
		// 	}
				
		// sleep(10);
		// printf("-----------------------------------------------------------\n");
		// printf("core %2d copy done !\n", spdk_env_get_current_core());
		memset(dst, 2, size);
		
	}
	// sleep(5);
	for (int i = 0; i < N; i++)
	{
		printf("core %d:a[%d] = %d\n", spdk_env_get_current_core(), i, a[i]);
	}
	return 0;
}

int main(int argc, char **argv)
{

	if (opts_init_env(argc, argv) < 0)
	{
		printf("opts init environment fail.\n");
		return 0;
	}
	else
		printf("opts init environment success.\n");

	uint64_t master_core = spdk_env_get_current_core(), timeuse;
	
	printf("master_core is %d in socket %d\n", master_core, spdk_env_get_socket_id(master_core));

	int i = 0;
	SPDK_ENV_FOREACH_CORE(i)
	{
		if (i != master_core)
			spdk_env_thread_launch_pinned(i, work_fn, NULL);
		else
			continue;
		// printf("%d\n",i);
	}
	
	while(spdk_env_thread_wait_all_one_time()!=0)
	// while (1)
	{
		// peek_ioat_comp_update();
		// printf("\n");
		ioat_poll();
	}
		

	ioat_cleanup();

	// printf("launch failed!%d\n",rc);

	// clock_t start_t, end_t;
	// start_t = clock();
	// clock_gettime(CLOCK_REALTIME, &start);

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


	// clock_gettime(CLOCK_REALTIME, &end);
	// timeuse=(end.tv_sec*pow(10,9)+end.tv_nsec)-(start.tv_sec*pow(10,9)+start.tv_nsec);
	// uint64_t speed=((double)worker->size*count/(1024*1024))/((double)timeuse/pow(10,9));
	// printf("%luns,%luMB/s\n",timeuse,speed);
}
