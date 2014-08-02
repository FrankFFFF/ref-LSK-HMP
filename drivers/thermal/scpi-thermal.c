#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/scpi_protocol.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/topology.h>

#define SOC_SENSOR "SENSOR_TEMP_SOC"

#define NUM_TRIPS 2
#define PASSIVE_INTERVAL 100
#define IDLE_INTERVAL 1000

#define NUM_CLUSTERS 2
enum cluster_type {
	CLUSTER_BIG = 0,
	CLUSTER_LITTLE
};

struct scpi_sensor {
	u16 sensor_id;
	unsigned long prev_temp;
	u32 alpha;
	struct thermal_zone_device *tzd;
	struct cpumask cluster[NUM_CLUSTERS];
	struct thermal_cooling_device *cdevs[NUM_CLUSTERS];
	int trip_temp[NUM_TRIPS];
};

struct scpi_sensor scpi_temp_sensor;

static int get_dyn_power_coeff(enum cluster_type cluster)
{
	int coeff = 0;

	switch(cluster) {
	case CLUSTER_BIG:
		coeff = 530;
		break;
	case CLUSTER_LITTLE:
		coeff = 140;
		break;
	}

	return coeff;
}

#define GPU_WEIGHT (1 << 8)

/*
 * The weight is an integer multiplied by 256.
 */
static u32 get_cluster_weight(enum cluster_type cluster)
{
	u32 weight = 0;

	switch(cluster) {
	case CLUSTER_BIG:
		weight = 1 * 256;
		break;
	case CLUSTER_LITTLE:
		weight = 2 * 256;
		break;
	}

	return weight;
}

static int get_cpu_static_power_coeff(enum cluster_type cluster)
{
	int coeff = 0;

	switch(cluster) {
	case CLUSTER_BIG:
		/* 75mW @ 85C/0.9V */
		coeff = 103;
		break;
	case CLUSTER_LITTLE:
		/* 26mW @ 85C/0.9V */
		coeff = 36;
		break;
	}

	return coeff;
}

static int get_cache_static_power_coeff(enum cluster_type cluster)
{
	int coeff = 0;

	switch(cluster) {
	case CLUSTER_BIG:
		/* 64mW @ 85C/0.9V */
		coeff = 88;
		break;
	case CLUSTER_LITTLE:
		/* 53mW @ 85C/0.9V */
		coeff = 73;
		break;
	}

	return coeff;
}

static unsigned long get_temperature_scale(unsigned long temp)
{
	int i, t_exp = 1, t_scale = 0;
	int coeff[] = { 32000, 4700, -80, 2 }; // * 1E6

	for (i = 0; i < 4; i++) {
		t_scale += coeff[i] * t_exp;
		t_exp *= temp;
	}

	return t_scale / 1000; // the value returned needs to be /1E3
}

static unsigned long get_voltage_scale(unsigned long u_volt)
{
	unsigned long m_volt = u_volt / 1000;
	unsigned long v_scale;

	v_scale = m_volt * m_volt * m_volt; // = (m_V^3) / (900 ^ 3) =

	return v_scale / 1000000; // the value returned needs to be /(1E3)
}

/* voltage in uV and temperature in mC */
static u32 get_static_power(cpumask_t *cpumask, unsigned long u_volt)
{
	unsigned long temperature, t_scale, v_scale;
	u32 cpu_coeff, mw_leakage;
	int nr_cpus = cpumask_weight(cpumask);
	enum cluster_type cluster =
		topology_physical_package_id(cpumask_any(cpumask));

	cpu_coeff = get_cpu_static_power_coeff(cluster);

	temperature = scpi_temp_sensor.tzd->temperature / 1000;

	t_scale = get_temperature_scale(temperature);
	v_scale = get_voltage_scale(u_volt);

	mw_leakage = nr_cpus * (cpu_coeff * t_scale * v_scale) / 1000000;

	if (nr_cpus) {
		u32 cache_coeff = get_cache_static_power_coeff(cluster);
		mw_leakage += (cache_coeff * v_scale * t_scale) / 1000000; /* cache leakage */
	}

	return mw_leakage;
}

#define FRAC_BITS 8
#define int_to_frac(x) ((x) << FRAC_BITS)
#define frac_to_int(x) ((x) >> FRAC_BITS)

/**
 * mul_frac() - multiply two fixed-point numbers
 * @x:	first multiplicand
 * @y:	second multiplicand
 *
 * Return: the result of multiplying two fixed-point numbers.  The
 * result is also a fixed-point number.
 */
static inline s64 mul_frac(s64 x, s64 y)
{
	return (x * y) >> FRAC_BITS;
}

static int get_temp_value(void *data, long *temp)
{
	struct scpi_sensor *sensor = (struct scpi_sensor *)data;
	u32 val;
	int ret;
	unsigned long est_temp;

	ret = scpi_get_sensor_value(sensor->sensor_id, &val);
	if (ret)
		return ret;

	if (!sensor->prev_temp)
		sensor->prev_temp = val;

	est_temp = mul_frac(sensor->alpha, val) +
		mul_frac((int_to_frac(1) - sensor->alpha), sensor->prev_temp);

	sensor->prev_temp = est_temp;
	*temp = est_temp;

	return 0;
}

extern struct dentry *power_allocator_d;

static void update_debugfs(struct scpi_sensor *sensor_data)
{
	struct dentry *dentry_f;

	dentry_f = debugfs_create_u32("alpha", S_IWUSR | S_IRUGO,
				power_allocator_d, &sensor_data->alpha);
	if (IS_ERR_OR_NULL(dentry_f)) {
		pr_warn("Unable to create debugfsfile: alpha\n");
		return;
	}

	dentry_f = debugfs_create_u32("sustainable_power", S_IWUSR | S_IRUGO,
				power_allocator_d,
				&sensor_data->tzd->tzp->sustainable_power);
	if (IS_ERR_OR_NULL(dentry_f)) {
		pr_warn("Unable to create debugfsfile: sustainable_power\n");
		return;
	}
}

static int scpi_get_temp(struct thermal_zone_device *tz, unsigned long *temp)
{
	return get_temp_value(tz->devdata, temp);
}

static int scpi_get_trip_type(struct thermal_zone_device *tz, int trip,
			enum thermal_trip_type *type)
{
	*type = THERMAL_TRIP_PASSIVE;

	return 0;
}

static int scpi_get_trip_temp(struct thermal_zone_device *tz, int trip,
			unsigned long *temp)
{
	struct scpi_sensor *sensor_data = tz->devdata;

	if ((trip < 0) || trip >= NUM_TRIPS)
		return -EINVAL;

	*temp = sensor_data->trip_temp[trip];
	return 0;
}

#define DECLARE_MATCH_CPU_FUNCTION(clus_name)			    \
static int match_##clus_name##_cdev(struct thermal_zone_device *tz, \
				struct thermal_cooling_device *cdev)	\
{									\
	struct cpufreq_cooling_device *cpufreq_cdev;			\
	struct cpumask *my_cpumask, *cdev_cpumask;			\
									\
	if (strncmp(cdev->type, "thermal-cpufreq-", strlen("thermal-cpufreq-"))) \
		return 1;						\
									\
	cpufreq_cdev = cdev->devdata;					\
	cdev_cpumask = &cpufreq_cdev->allowed_cpus;			\
	my_cpumask = &scpi_temp_sensor.cluster[clus_name];		\
									\
	return !cpumask_equal(my_cpumask, cdev_cpumask);		\
}

DECLARE_MATCH_CPU_FUNCTION(CLUSTER_LITTLE)
DECLARE_MATCH_CPU_FUNCTION(CLUSTER_BIG)

static int match_gpu_devfreq_cdev(struct thermal_zone_device *tz,
				struct thermal_cooling_device *cdev)
{
	return strcmp(cdev->type, "devfreq");
}

static int (*match_cpu_cdev[]) (struct thermal_zone_device *,
			struct thermal_cooling_device *) = {
	[CLUSTER_BIG] = match_CLUSTER_BIG_cdev,
	[CLUSTER_LITTLE] = match_CLUSTER_LITTLE_cdev,
};

static struct thermal_zone_device_ops scpi_tz_ops = {
	.get_temp = scpi_get_temp,
	.get_trip_type = scpi_get_trip_type,
	.get_trip_temp = scpi_get_trip_temp,
};

static struct thermal_zone_params scpi_tz_params = {
	.sustainable_power = 2500,
	.num_tbps = 3,
};

static int scpi_thermal_probe(struct platform_device *pdev)
{
	struct scpi_sensor *sensor_data = &scpi_temp_sensor;
	struct device_node *np;
	struct thermal_bind_params *tbp;
	int sensor, cpu;
	int i;

	if (!cpufreq_frequency_get_table(0)) {
		dev_info(&pdev->dev,
			"Frequency table not initialized. Deferring probe...\n");
		return -EPROBE_DEFER;
	}

	platform_set_drvdata(pdev, sensor_data);

	tbp = kcalloc(scpi_tz_params.num_tbps, sizeof(*tbp), GFP_KERNEL);
	if (!tbp)
		return -ENOMEM;

	for_each_possible_cpu(cpu) {
		int cluster_id = topology_physical_package_id(cpu);
		if (cluster_id > NUM_CLUSTERS) {
			pr_warn("Cluster id: %d > %d\n", cluster_id, NUM_CLUSTERS);
			goto error;
		}

		cpumask_set_cpu(cpu, &sensor_data->cluster[cluster_id]);
	}

	for (i = 0; i < NUM_CLUSTERS; i++) {
		char node[16];
		enum cluster_type cluster =
			topology_physical_package_id(cpumask_any(&sensor_data->cluster[i]));

		snprintf(node, 16, "cluster%d", i);
		np = of_find_node_by_name(NULL, node);

		if (!np)
			dev_info(&pdev->dev, "Node not found: %s\n", node);

		sensor_data->cdevs[i] =
			of_cpufreq_power_cooling_register(np,
						&sensor_data->cluster[i],
						get_dyn_power_coeff(cluster),
						get_static_power);

		if (IS_ERR(sensor_data->cdevs[i]))
			dev_warn(&pdev->dev,
				"Error registering cooling device: %d\n", i);

		tbp[i].match = match_cpu_cdev[cluster];
		tbp[i].trip_mask = 1 << 1;
		tbp[i].weight = get_cluster_weight(cluster);
	}

	BUG_ON(i >= scpi_tz_params.num_tbps);
	tbp[i].match = match_gpu_devfreq_cdev;
	tbp[i].trip_mask = 1 << 1;
	tbp[i].weight = GPU_WEIGHT;

	if ((sensor = scpi_get_sensor(SOC_SENSOR)) < 0) {
		dev_warn(&pdev->dev, "%s not found. ret=%d\n", SOC_SENSOR, sensor);
		goto error;
	}

	sensor_data->sensor_id = (u16)sensor;
	dev_info(&pdev->dev, "Probed %s sensor. Id=%hu\n", SOC_SENSOR, sensor_data->sensor_id);

	sensor_data->trip_temp[0] = 60000;
	sensor_data->trip_temp[1] = 65000;

	/*
	 * alpha ~= 2 / (N + 1) with N the window of a rolling mean We
	 * use 8-bit fixed point arithmetic.  For a rolling average of
	 * window 20, alpha = 2 / (20 + 1) ~= 0.09523809523809523 .
	 * In 8-bit fixed point arigthmetic, 0.09523809523809523 * 256
	 * ~= 24
	 */
	sensor_data->alpha = 24;

	scpi_tz_params.tbp = tbp;

	sensor_data->tzd = thermal_zone_device_register("scpi_thermal",
							NUM_TRIPS,
							0, sensor_data,
							&scpi_tz_ops,
							&scpi_tz_params,
							PASSIVE_INTERVAL,
							IDLE_INTERVAL);

	if (IS_ERR(sensor_data->tzd)) {
		dev_warn(&pdev->dev, "Error registering sensor: %p\n", sensor_data->tzd);
		return PTR_ERR(sensor_data->tzd);
	}

	update_debugfs(sensor_data);

	thermal_zone_device_update(sensor_data->tzd);

	return 0;

error:
	return -ENODEV;
}

static int scpi_thermal_remove(struct platform_device *pdev)
{
	struct scpi_sensor *sensor = platform_get_drvdata(pdev);

	thermal_zone_device_unregister(sensor->tzd);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct of_device_id scpi_thermal_of_match[] = {
	{ .compatible = "arm,scpi-thermal" },
	{},
};
MODULE_DEVICE_TABLE(of, scpi_thermal_of_match);

static struct platform_driver scpi_thermal_platdrv = {
	.driver = {
		.name		= "scpi-thermal",
		.owner		= THIS_MODULE,
		.of_match_table = scpi_thermal_of_match,
	},
	.probe	= scpi_thermal_probe,
	.remove	= scpi_thermal_remove,
};
module_platform_driver(scpi_thermal_platdrv);

MODULE_LICENSE("GPL");
