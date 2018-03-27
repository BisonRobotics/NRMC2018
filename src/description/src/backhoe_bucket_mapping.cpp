#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

ros::Publisher *pub;

const double backhoe_positions[] = {0.000000,0.003145,0.006289,0.009434,0.012579,0.015724,0.018868,0.022013,0.025158,0.028303,0.031447,0.034592,0.037737,0.040882,0.044026,0.047171,0.050316,0.053461,0.056605,0.059750,0.062895,0.066039,0.069184,0.072329,0.075474,0.078618,0.081763,0.084908,0.088053,0.091197,0.094342,0.097487,0.100632,0.103776,0.106921,0.110066,0.113211,0.116355,0.119500,0.122645,0.125789,0.128934,0.132079,0.135224,0.138368,0.141513,0.144658,0.147803,0.150947,0.154092,0.157237,0.160382,0.163526,0.166671,0.169816,0.172961,0.176105,0.179250,0.182395,0.185540,0.188684,0.191829,0.194974,0.198118,0.201263,0.204408,0.207553,0.210697,0.213842,0.216987,0.220132,0.223276,0.226421,0.229566,0.232711,0.235855,0.239000,0.242145,0.245290,0.248434,0.251579,0.254724,0.257868,0.261013,0.264158,0.267303,0.270447,0.273592,0.276737,0.279882,0.283026,0.286171,0.289316,0.292461,0.295605,0.298750,0.301895,0.305040,0.308184,0.311329,0.314474,0.317618,0.320763,0.323908,0.327053,0.330197,0.333342,0.336487,0.339632,0.342776,0.345921,0.349066,0.352211,0.355355,0.358500,0.361645,0.364790,0.367934,0.371079,0.374224,0.377368,0.380513,0.383658,0.386803,0.389947,0.393092,0.396237,0.399382,0.402526,0.405671,0.408816,0.411961,0.415105,0.418250,0.421395,0.424540,0.427684,0.430829,0.433974,0.437118,0.440263,0.443408,0.446553,0.449697,0.452842,0.455987,0.459132,0.462276,0.465421,0.468566,0.471711,0.474855,0.478000,0.481145,0.484290,0.487434,0.490579,0.493724,0.496869,0.500013,0.503158,0.506303,0.509447,0.512592,0.515737,0.518882,0.522026,0.525171,0.528316,0.531461,0.534605,0.537750,0.540895,0.544040,0.547184,0.550329,0.553474,0.556619,0.559763,0.562908,0.566053,0.569197,0.572342,0.575487,0.578632,0.581776,0.584921,0.588066,0.591211,0.594355,0.597500,0.600645,0.603790,0.606934,0.610079,0.613224,0.616369,0.619513,0.622658,0.625803,0.628947,0.632092,0.635237,0.638382,0.641526,0.644671,0.647816,0.650961,0.654105,0.657250,0.660395,0.663540,0.666684,0.669829,0.672974,0.676119,0.679263,0.682408,0.685553,0.688697,0.691842,0.694987,0.698132,0.701276,0.704421,0.707566,0.710711,0.713855,0.717000,0.720145,0.723290,0.726434,0.729579,0.732724,0.735869,0.739013,0.742158,0.745303,0.748447,0.751592,0.754737,0.757882,0.761026,0.764171,0.767316,0.770461,0.773605,0.776750,0.779895,0.783040,0.786184,0.789329,0.792474,0.795619,0.798763,0.801908,0.805053,0.808198,0.811342,0.814487,0.817632,0.820776,0.823921,0.827066,0.830211,0.833355,0.836500,0.839645,0.842790,0.845934,0.849079,0.852224,0.855369,0.858513,0.861658,0.864803,0.867948,0.871092,0.874237,0.877382,0.880526,0.883671,0.886816,0.889961,0.893105,0.896250,0.899395,0.902540,0.905684,0.908829,0.911974,0.915119,0.918263,0.921408,0.924553,0.927698,0.930842,0.933987,0.937132,0.940276,0.943421,0.946566,0.949711,0.952855,0.956000,0.959145,0.962290,0.965434,0.968579,0.971724,0.974869,0.978013,0.981158,0.984303,0.987448,0.990592,0.993737,0.996882,1.000026,1.003171,1.006316,1.009461,1.012605,1.015750,1.018895,1.022040,1.025184,1.028329,1.031474,1.034619,1.037763,1.040908,1.044053,1.047198,1.050342,1.053487,1.056632,1.059777,1.062921,1.066066,1.069211,1.072355,1.075500,1.078645,1.081790,1.084934,1.088079,1.091224,1.094369,1.097513,1.100658,1.103803,1.106948,1.110092,1.113237,1.116382,1.119527,1.122671,1.125816,1.128961,1.132105,1.135250,1.138395,1.141540,1.144684,1.147829,1.150974,1.154119,1.157263,1.160408,1.163553,1.166698,1.169842,1.172987,1.176132,1.179277,1.182421,1.185566,1.188711,1.191855,1.195000,1.198145,1.201290,1.204434,1.207579,1.210724,1.213869,1.217013,1.220158,1.223303,1.226448,1.229592,1.232737,1.235882,1.239027,1.242171,1.245316,1.248461,1.251605,1.254750,1.257895,1.261040,1.264184,1.267329,1.270474,1.273619,1.276763,1.279908,1.283053,1.286198,1.289342,1.292487,1.295632,1.298777,1.301921,1.305066,1.308211,1.311355,1.314500,1.317645,1.320790,1.323934,1.327079,1.330224,1.333369,1.336513,1.339658,1.342803,1.345948,1.349092,1.352237,1.355382,1.358527,1.361671,1.364816,1.367961,1.371106,1.374250,1.377395,1.380540,1.383684,1.386829,1.389974,1.393119,1.396263,1.399408,1.402553,1.405698,1.408842,1.411987,1.415132,1.418277,1.421421,1.424566,1.427711,1.430856,1.434000,1.437145,1.440290,1.443434,1.446579,1.449724,1.452869,1.456013,1.459158,1.462303,1.465448,1.468592,1.471737,1.474882,1.478027,1.481171,1.484316,1.487461,1.490606,1.493750,1.496895,1.500040,1.503184,1.506329,1.509474,1.512619,1.515763,1.518908,1.522053,1.525198,1.528342,1.531487,1.534632,1.537777,1.540921,1.544066,1.547211,1.550356,1.553500,1.556645,1.559790,1.562934,1.566079,1.569224,1.572369,1.575513,1.578658,1.581803,1.584948,1.588092,1.591237,1.594382,1.597527,1.600671,1.603816,1.606961,1.610106,1.613250,1.616395,1.619540,1.622684,1.625829,1.628974,1.632119,1.635263,1.638408,1.641553,1.644698,1.647842,1.650987,1.654132,1.657277,1.660421,1.663566,1.666711,1.669856,1.673000,1.676145,1.679290,1.682435,1.685579,1.688724,1.691869,1.695013,1.698158,1.701303,1.704448,1.707592,1.710737,1.713882,1.717027,1.720171,1.723316,1.726461,1.729606,1.732750,1.735895,1.739040,1.742185,1.745329,1.748474,1.751619,1.754763,1.757908,1.761053,1.764198,1.767342,1.770487,1.773632,1.776777,1.779921,1.783066,1.786211,1.789356,1.792500,1.795645,1.798790,1.801935,1.805079,1.808224,1.811369,1.814513,1.817658,1.820803,1.823948,1.827092,1.830237,1.833382,1.836527,1.839671,1.842816,1.845961,1.849106,1.852250,1.855395,1.858540,1.861685,1.864829,1.867974,1.871119,1.874263,1.877408,1.880553,1.883698,1.886842,1.889987,1.893132,1.896277,1.899421,1.902566,1.905711,1.908856,1.912000,1.915145,1.918290,1.921435,1.924579,1.927724,1.930869,1.934013,1.937158,1.940303,1.943448,1.946592,1.949737,1.952882,1.956027,1.959171,1.962316,1.965461,1.968606,1.971750,1.974895,1.978040,1.981185,1.984329,1.987474,1.990619,1.993764,1.996908,2.000053,2.003198,2.006342,2.009487,2.012632,2.015777,2.018921,2.022066,2.025211,2.028356,2.031500,2.034645,2.037790,2.040935,2.044079,2.047224,2.050369,2.053514,2.056658,2.059803,2.062948,2.066092,2.069237,2.072382,2.075527,2.078671,2.081816,2.084961,2.088106,2.091250,2.094395,2.097540,2.100685,2.103829,2.106974,2.110119,2.113264,2.116408,2.119553,2.122698,2.125842,2.128987,2.132132,2.135277,2.138421,2.141566,2.144711,2.147856,2.151000,2.154145,2.157290,2.160435,2.163579,2.166724,2.169869,2.173014,2.176158,2.179303,2.182448,2.185592,2.188737,2.191882,2.195027,2.198171,2.201316,2.204461,2.207606,2.210750,2.213895,2.217040,2.220185,2.223329,2.226474,2.229619,2.232764,2.235908,2.239053,2.242198,2.245342,2.248487,2.251632,2.254777,2.257921,2.261066,2.264211,2.267356,2.270500,2.273645,2.276790,2.279935,2.283079,2.286224,2.289369,2.292514,2.295658,2.298803,2.301948,2.305093,2.308237,2.311382,2.314527,2.317671,2.320816,2.323961,2.327106,2.330250,2.333395,2.336540,2.339685,2.342829,2.345974,2.349119,2.352264,2.355408,2.358553,2.361698,2.364843,2.367987,2.371132,2.374277,2.377421,2.380566,2.383711,2.386856,2.390000,2.393145,2.396290,2.399435,2.402579,2.405724,2.408869,2.412014,2.415158,2.418303,2.421448,2.424593,2.427737,2.430882,2.434027,2.437171,2.440316,2.443461,2.446606,2.449750,2.452895,2.456040,2.459185,2.462329,2.465474,2.468619,2.471764,2.474908,2.478053,2.481198,2.484343,2.487487,2.490632,2.493777,2.496921,2.500066,2.503211,2.506356,2.509500,2.512645,2.515790,2.518935,2.522079,2.525224,2.528369,2.531514,2.534658,2.537803,2.540948,2.544093,2.547237,2.550382,2.553527,2.556671,2.559816,2.562961,2.566106,2.569250,2.572395,2.575540,2.578685,2.581829,2.584974,2.588119,2.591264,2.594408,2.597553,2.600698,2.603843,2.606987,2.610132,2.613277,2.616422,2.619566,2.622711,2.625856,2.629000,2.632145,2.635290,2.638435,2.641579,2.644724,2.647869,2.651014,2.654158,2.657303,2.660448,2.663593,2.666737,2.669882,2.673027,2.676172,2.679316,2.682461,2.685606,2.688750,2.691895,2.695040,2.698185,2.701329,2.704474,2.707619,2.710764,2.713908,2.717053,2.720198,2.723343,2.726487,2.729632,2.732777,2.735922,2.739066,2.742211,2.745356,2.748500,2.751645,2.754790,2.757935,2.761079,2.764224,2.767369,2.770514,2.773658,2.776803,2.779948,2.783093,2.786237,2.789382,2.792527,2.795672,2.798816,2.801961,2.805106,2.808250,2.811395,2.814540,2.817685,2.820829,2.823974,2.827119,2.830264,2.833408,2.836553,2.839698,2.842843,2.845987,2.849132,2.852277,2.855422,2.858566,2.861711,2.864856,2.868001,2.871145,2.874290,2.877435,2.880579,2.883724,2.886869,2.890014,2.893158,2.896303,2.899448,2.902593,2.905737,2.908882,2.912027,2.915172,2.918316,2.921461,2.924606,2.927751,2.930895,2.934040,2.937185,2.940329,2.943474,2.946619,2.949764,2.952908,2.956053,2.959198,2.962343,2.965487,2.968632,2.971777,2.974922,2.978066,2.981211,2.984356,2.987501,2.990645,2.993790,2.996935,3.000079,3.003224,3.006369,3.009514,3.012658,3.015803,3.018948,3.022093,3.025237,3.028382,3.031527,3.034672,3.037816,3.040961,3.044106,3.047251,3.050395,3.053540,3.056685,3.059829,3.062974,3.066119,3.069264,3.072408,3.075553,3.078698,3.081843,3.084987,3.088132,3.091277,3.094422,3.097566,3.100711,3.103856,3.107001,3.110145,3.113290,3.116435,3.119579,3.122724,3.125869,3.129014,3.132158,3.135303,3.138448,3.141593};
const double bucket_positions[] = {1.329597,1.333907,1.338212,1.342514,1.346812,1.351106,1.355396,1.359683,1.363966,1.368245,1.372521,1.376794,1.381063,1.385328,1.389590,1.393849,1.398105,1.402357,1.406606,1.410852,1.415094,1.419334,1.423571,1.427804,1.432034,1.436262,1.440487,1.444708,1.448927,1.453143,1.457356,1.461567,1.465775,1.469980,1.474182,1.478382,1.482579,1.486774,1.490966,1.495155,1.499342,1.503527,1.507709,1.511889,1.516066,1.520241,1.524414,1.528585,1.532753,1.536919,1.541083,1.545244,1.549404,1.553561,1.557716,1.561869,1.566020,1.570169,1.574316,1.578461,1.582604,1.586745,1.590884,1.595021,1.599156,1.603290,1.607421,1.611551,1.615679,1.619805,1.623929,1.628052,1.632173,1.636292,1.640409,1.644525,1.648639,1.652752,1.656862,1.660972,1.665079,1.669185,1.673290,1.677392,1.681494,1.685594,1.689692,1.693789,1.697884,1.701978,1.706070,1.710161,1.714251,1.718339,1.722426,1.726511,1.730595,1.734678,1.738759,1.742839,1.746918,1.750996,1.755072,1.759146,1.763220,1.767292,1.771363,1.775433,1.779501,1.783568,1.787634,1.791699,1.795763,1.799825,1.803886,1.807946,1.812005,1.816063,1.820120,1.824175,1.828229,1.832282,1.836334,1.840385,1.844435,1.848484,1.852531,1.856578,1.860623,1.864667,1.868710,1.872752,1.876793,1.880833,1.884872,1.888910,1.892947,1.896983,1.901017,1.905051,1.909083,1.913115,1.917145,1.921175,1.925203,1.929230,1.933257,1.937282,1.941306,1.945329,1.949351,1.953372,1.957392,1.961412,1.965429,1.969446,1.973462,1.977477,1.981491,1.985504,1.989516,1.993526,1.997536,2.001544,2.005552,2.009558,2.013564,2.017568,2.021571,2.025573,2.029574,2.033574,2.037573,2.041571,2.045568,2.049563,2.053558,2.057551,2.061543,2.065534,2.069524,2.073513,2.077500,2.081487,2.085472,2.089456,2.093439,2.097421,2.101401,2.105380,2.109358,2.113335,2.117310,2.121284,2.125257,2.129229,2.133199,2.137168,2.141135,2.145102,2.149066,2.153030,2.156992,2.160953,2.164912,2.168870,2.172826,2.176781,2.180734,2.184686,2.188636,2.192585,2.196532,2.200478,2.204422,2.208364,2.212305,2.216244,2.220181,2.224116,2.228050,2.231982,2.235913,2.239841,2.243768,2.247692,2.251615,2.255536,2.259455,2.263372,2.267287,2.271200,2.275111,2.279020,2.282926,2.286831,2.290733,2.294633,2.298531,2.302426,2.306320,2.310210,2.314099,2.317985,2.321868,2.325749,2.329628,2.333503,2.337376,2.341247,2.345115,2.348980,2.352842,2.356701,2.360557,2.364411,2.368261,2.372108,2.375953,2.379794,2.383631,2.387466,2.391297,2.395125,2.398949,2.402770,2.406587,2.410401,2.414211,2.418017,2.421819,2.425618,2.429412,2.433202,2.436989,2.440771,2.444549,2.448322,2.452091,2.455856,2.459616,2.463372,2.467122,2.470868,2.474609,2.478345,2.482076,2.485801,2.489522,2.493236,2.496946,2.500650,2.504348,2.508040,2.511726,2.515407,2.519081,2.522749,2.526410,2.530065,2.533713,2.537355,2.540989,2.544617,2.548237,2.551850,2.555456,2.559054,2.562644,2.566227,2.569801,2.573367,2.576924,2.580473,2.584013,2.587545,2.591067,2.594580,2.598083,2.601577,2.605061,2.608534,2.611998,2.615451,2.618893,2.622324,2.625744,2.629153,2.632550,2.635935,2.639308,2.642669,2.646016,2.649351,2.652673,2.655981,2.659276,2.662556,2.665822,2.669074,2.672310,2.675531,2.678736,2.681926,2.685099,2.688255,2.691395,2.694516,2.697621,2.700706,2.703774,2.706822,2.709851,2.712860,2.715849,2.718817,2.721764,2.724690,2.727593,2.730474,2.733332,2.736166,2.738977,2.741762,2.744523,2.747258,2.749967,2.752649,2.755304,2.757931,2.760530,2.763100,2.765640,2.768149,2.770628,2.773076,2.775491,2.777873,2.780222,2.782537,2.784817,2.787062,2.789270,2.791442,2.793576,2.795672,2.797728,2.799746,2.801722,2.803658,2.805552,2.807403,2.809212,2.810976,2.812695,2.814369,2.815997,2.817579,2.819112,2.820598,2.822035,2.823422,2.824759,2.826045,2.827280,2.828462,2.829592,2.830669,2.831692,2.832661,2.833575,2.834433,2.835236,2.835982,2.836672,2.837305,2.837880,2.838397,2.838857,2.839258,2.839601,2.839884,2.840109,2.840275,2.840382,2.840430,2.840418,2.840347,2.840217,2.840028,2.839780,2.839473,2.839107,2.838683,2.838200,2.837660,2.837062,2.836407,2.835695,2.834926,2.834102,2.833221,2.832286,2.831296,2.830251,2.829154,2.828003,2.826800,2.825544,2.824238,2.822881,2.821475,2.820019,2.818514,2.816961,2.815362,2.813715,2.812023,2.810286,2.808505,2.806679,2.804811,2.802901,2.800949,2.798956,2.796923,2.794851,2.792740,2.790591,2.788405,2.786182,2.783923,2.781629,2.779301,2.776939,2.774543,2.772115,2.769656,2.767164,2.764643,2.762091,2.759510,2.756900,2.754262,2.751596,2.748903,2.746184,2.743439,2.740668,2.737873,2.735053,2.732209,2.729342,2.726452,2.723540,2.720606,2.717651,2.714674,2.711677,2.708660,2.705624,2.702568,2.699493,2.696400,2.693289,2.690160,2.687014,2.683851,2.680672,2.677476,2.674264,2.671037,2.667795,2.664538,2.661266,2.657980,2.654680,2.651366,2.648039,2.644699,2.641346,2.637981,2.634603,2.631213,2.627812,2.624398,2.620974,2.617538,2.614092,2.610635,2.607167,2.603690,2.600202,2.596704,2.593197,2.589681,2.586155,2.582620,2.579076,2.575524,2.571963,2.568394,2.564816,2.561231,2.557638,2.554037,2.550428,2.546812,2.543189,2.539559,2.535921,2.532277,2.528626,2.524969,2.521305,2.517634,2.513958,2.510275,2.506586,2.502892,2.499191,2.495485,2.491774,2.488057,2.484334,2.480607,2.476874,2.473136,2.469393,2.465645,2.461893,2.458136,2.454374,2.450607,2.446837,2.443061,2.439282,2.435498,2.431710,2.427918,2.424122,2.420322,2.416518,2.412710,2.408899,2.405084,2.401265,2.397443,2.393617,2.389788,2.385956,2.382120,2.378281,2.374439,2.370593,2.366745,2.362893,2.359039,2.355181,2.351321,2.347458,2.343592,2.339723,2.335851,2.331977,2.328100,2.324221,2.320339,2.316454,2.312567,2.308678,2.304786,2.300892,2.296996,2.293097,2.289196,2.285293,2.281388,2.277480,2.273571,2.269659,2.265745,2.261829,2.257912,2.253992,2.250070,2.246147,2.242221,2.238294,2.234365,2.230434,2.226501,2.222566,2.218630,2.214692,2.210753,2.206811,2.202868,2.198924,2.194978,2.191030,2.187080,2.183130,2.179177,2.175223,2.171268,2.167311,2.163353,2.159393,2.155432,2.151469,2.147505,2.143540,2.139573,2.135605,2.131635,2.127665,2.123692,2.119719,2.115744,2.111769,2.107791,2.103813,2.099833,2.095852,2.091870,2.087887,2.083903,2.079917,2.075930,2.071942,2.067953,2.063962,2.059971,2.055978,2.051985,2.047990,2.043994,2.039997,2.035998,2.031999,2.027999,2.023997,2.019995,2.015991,2.011986,2.007980,2.003973,1.999966,1.995957,1.991947,1.987935,1.983923,1.979910,1.975896,1.971881,1.967864,1.963847,1.959829,1.955809,1.951789,1.947767,1.943745,1.939721,1.935696,1.931671,1.927644,1.923616,1.919588,1.915558,1.911527,1.907495,1.903462,1.899428,1.895393,1.891357,1.887320,1.883282,1.879242,1.875202,1.871160,1.867118,1.863074,1.859030,1.854984,1.850937,1.846889,1.842840,1.838790,1.834738,1.830686,1.826632,1.822578,1.818522,1.814465,1.810407,1.806347,1.802287,1.798225,1.794162,1.790098,1.786033,1.781967,1.777899,1.773830,1.769760,1.765688,1.761615,1.757541,1.753466,1.749390,1.745312,1.741233,1.737152,1.733070,1.728987,1.724902,1.720816,1.716729,1.712640,1.708550,1.704459,1.700366,1.696271,1.692175,1.688078,1.683979,1.679879,1.675777,1.671673,1.667568,1.663461,1.659353,1.655243,1.651132,1.647019,1.642904,1.638788,1.634670,1.630550,1.626428,1.622305,1.618180,1.614053,1.609925,1.605794,1.601662,1.597528,1.593392,1.589254,1.585114,1.580972,1.576829,1.572683,1.568535,1.564385,1.560234,1.556080,1.551924,1.547766,1.543605,1.539443,1.535278,1.531111,1.526942,1.522771,1.518597,1.514421,1.510243,1.506062,1.501879,1.497693,1.493505,1.489315,1.485122,1.480926,1.476728,1.472527,1.468324,1.464118,1.459909,1.455697,1.451483,1.447266,1.443046,1.438823,1.434597,1.430369,1.426137,1.421902,1.417664,1.413424,1.409180,1.404933,1.400682,1.396429,1.392172,1.387912,1.383648,1.379382,1.375111,1.370837,1.366560,1.362279,1.357995,1.353707,1.349415,1.345119,1.340820,1.336517,1.332210,1.327899,1.323583,1.319264,1.314941,1.310614,1.306283,1.301947,1.297607,1.293262,1.288914,1.284560,1.280203,1.275840,1.271473,1.267102,1.262725,1.258344,1.253958,1.249567,1.245171,1.240770,1.236364,1.231952,1.227535,1.223113,1.218686,1.214253,1.209814,1.205370,1.200920,1.196465,1.192003,1.187536,1.183063,1.178583,1.174097,1.169605,1.165107,1.160602,1.156091,1.151573,1.147049,1.142517,1.137979,1.133434,1.128881,1.124322,1.119755,1.115181,1.110599,1.106010,1.101413,1.096808,1.092195,1.087574,1.082945,1.078308,1.073662,1.069008,1.064345,1.059674,1.054993,1.050303,1.045605,1.040896,1.036179,1.031452,1.026715,1.021968,1.017211,1.012444,1.007666,1.002878,0.998079,0.993270,0.988449,0.983617,0.978774,0.973919,0.969052,0.964173,0.959282,0.954379,0.949463,0.944535,0.939593,0.934638,0.929670,0.924687,0.919691,0.914681,0.909656,0.904617,0.899563,0.894493,0.889408,0.884307,0.879190,0.874057,0.868907,0.863740,0.858556,0.853354,0.848134,0.842895,0.837638,0.832362,0.827066,0.821751,0.816415,0.811058,0.805681,0.800282,0.794860,0.789417,0.783950,0.778460,0.772946,0.767407,0.761843,0.756254,0.750638,0.744995,0.739325,0.733627,0.727900,0.722143,0.716356,0.710538,0.704688,0.698806,0.692890,0.686939,0.680953,0.674931,0.668872,0.662774,0.656637,0.650460,0.644241,0.637978,0.631672,0.625320,0.618920,0.612473,0.605975,0.599425,0.592822,0.586163};

using sensor_msgs::JointState;

void jointStatesCallback(const JointState::ConstPtr& msg)
{
  for (int i = 0; i < msg->name.size(); i++)
  {
    if (msg->name[i] == "central_drive_to_monoboom")
    {
      // Look for closest match to position
      double current_min = M_2_PI;
      int current_min_index = 0;
      for (int j = 0; j < 1000; j++)
      {
        double diff = fabs(msg->position[i] - (backhoe_positions[j]));
        if (diff < current_min)
        {
          current_min = diff;
          current_min_index = j;
        }
      }
      // Output bucket joint state accordingly
      JointState bucket_joint_state;
      bucket_joint_state.name.emplace_back("frame_to_bucket");
      bucket_joint_state.position.push_back(bucket_positions[current_min_index]);
      bucket_joint_state.effort.push_back(0.0); // TODO Add
      bucket_joint_state.velocity.push_back(0.0); // TODO Add
      bucket_joint_state.header = msg->header;
      pub->publish(bucket_joint_state);
      ROS_INFO("Backhoe: %f, Bucket: %f", msg->position[i], bucket_joint_state.position[0]);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "backhoe_bucket_mapping");
  ros::NodeHandle n;
  pub = new ros::Publisher(n.advertise<sensor_msgs::JointState>("bucket_joint_states", 1000));
  ros::Subscriber sub = n.subscribe("joint_states", 1000, jointStatesCallback);
  ros::spin();

  return 0;
}