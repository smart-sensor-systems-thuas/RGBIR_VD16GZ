// SPDX-License-Identifier: MIT
/*
 * vd16gz-bridge: turn the ST VD16GZ RGB-IR sensor into ordinary V4L2 cameras.
 *
 * The vd56g3 kernel driver delivers the VD16GZ 4x4 RGB-IR mosaic as a 10-bit
 * greyscale stream on the Raspberry Pi 5 CSI-2 receiver (rp1-cfe). This daemon
 * configures that pipeline, splits every frame into RGB and IR images and writes
 * them to v4l2loopback devices, so any V4L2 application (OpenCV, GStreamer,
 * ffmpeg, ...) can open them like a webcam:
 *
 *   /dev/vd16gz-rgb   RGB24, colour image
 *   /dev/vd16gz-ir    GREY,  near-infrared image
 *   /dev/vd16gz-raw   Y16,   unprocessed mosaic (10-bit values 0..1023)
 *
 * RGB, IR and raw are made from the same sensor frame and carry the sensor's
 * capture timestamp, so frames from the different devices can be paired exactly.
 * Exposure and gain stay standard V4L2 controls on the sensor (/dev/vd16gz-sensor).
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>

#define W 1124
#define H 1364
#define NCAP 4
#define NOUT 4
#define MAX_THREADS 16

enum { R, G, B, I };

/* 4x4 colour filter layout at full resolution, no flips, top-left pixel first. */
static const char *CFA_ROWS[4] = { "GIGI", "BGRG", "GIGI", "RGBG" };

static struct {
	int half, linear, swap_rb, threads, black, verbose;
	float ir_subtract;
	int wb_mode; /* 0 none, 1 auto, 2 fixed */
	float wb_fixed[3];
	float ccm[3][4];
	int have_ccm;
	const char *dev_path[3]; /* rgb, ir, raw */
	const char *exposure;    /* NULL (leave), "auto" or lines */
	int gain;
} opt = {
	.threads = 4, .black = 64, .wb_mode = -1, .gain = -1,
	.dev_path = { "/dev/vd16gz-rgb", "/dev/vd16gz-ir", "/dev/vd16gz-raw" },
};

static volatile sig_atomic_t stop;

static void die(const char *fmt, ...) __attribute__((format(printf, 1, 2), noreturn));
static void die(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	fprintf(stderr, "vd16gz-bridge: ");
	vfprintf(stderr, fmt, ap);
	if (errno)
		fprintf(stderr, " (%s)", strerror(errno));
	fprintf(stderr, "\n");
	va_end(ap);
	exit(1);
}

static int xioctl(int fd, unsigned long req, void *arg)
{
	int r;
	do
		r = ioctl(fd, req, arg);
	while (r < 0 && errno == EINTR);
	return r;
}

/* ---------------------------------------------------------------- pipeline */

struct entity {
	uint32_t id;
	char name[64];
	char devnode[128];
};

static void devnode_path(uint32_t major, uint32_t minor, char *out, size_t n)
{
	char p[96], line[128];
	snprintf(p, sizeof(p), "/sys/dev/char/%u:%u/uevent", major, minor);
	out[0] = 0;
	FILE *f = fopen(p, "r");
	if (!f)
		return;
	while (fgets(line, sizeof(line), f))
		if (!strncmp(line, "DEVNAME=", 8)) {
			line[strcspn(line, "\n")] = 0;
			snprintf(out, n, "/dev/%s", line + 8);
		}
	fclose(f);
}

static int find_entity(int mfd, const char *prefix, struct entity *e)
{
	struct media_entity_desc d = { .id = MEDIA_ENT_ID_FLAG_NEXT };
	while (!xioctl(mfd, MEDIA_IOC_ENUM_ENTITIES, &d)) {
		if (!strncmp(d.name, prefix, strlen(prefix))) {
			e->id = d.id;
			snprintf(e->name, sizeof(e->name), "%s", d.name);
			devnode_path(d.dev.major, d.dev.minor, e->devnode, sizeof(e->devnode));
			return 0;
		}
		d.id |= MEDIA_ENT_ID_FLAG_NEXT;
	}
	return -1;
}

static void setup_link(int mfd, uint32_t src, uint16_t src_pad, uint32_t sink, uint16_t sink_pad, int enable)
{
	struct media_link_desc l = {
		.source = { .entity = src, .index = src_pad, .flags = MEDIA_PAD_FL_SOURCE },
		.sink = { .entity = sink, .index = sink_pad, .flags = MEDIA_PAD_FL_SINK },
		.flags = enable ? MEDIA_LNK_FL_ENABLED : 0,
	};
	if (xioctl(mfd, MEDIA_IOC_SETUP_LINK, &l) && enable)
		die("cannot enable media link %u:%u -> %u:%u", src, src_pad, sink, sink_pad);
}

static void set_subdev_fmt(const char *node, uint32_t pad)
{
	int fd = open(node, O_RDWR);
	if (fd < 0)
		die("cannot open %s", node);
	struct v4l2_subdev_format f = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = pad,
		.format = {
			.width = W, .height = H, .code = MEDIA_BUS_FMT_Y10_1X10,
			.field = V4L2_FIELD_NONE, .colorspace = V4L2_COLORSPACE_RAW,
			.quantization = V4L2_QUANTIZATION_FULL_RANGE,
		},
	};
	if (xioctl(fd, VIDIOC_SUBDEV_S_FMT, &f) || f.format.width != W || f.format.height != H)
		die("cannot set %ux%u Y10 on %s pad %u", W, H, node, pad);
	close(fd);
}

static void set_ctrl(int fd, uint32_t id, int32_t value, const char *name)
{
	struct v4l2_control c = { .id = id, .value = value };
	if (xioctl(fd, VIDIOC_S_CTRL, &c))
		fprintf(stderr, "vd16gz-bridge: warning: cannot set %s=%d: %s\n", name, value, strerror(errno));
}

/* Configure sensor -> csi2 -> rp1-cfe-csi2_ch0 for full-resolution Y10 and return the video node. */
static void setup_pipeline(char *video, size_t n)
{
	char path[32];
	for (int i = 0; i < 16; i++) {
		snprintf(path, sizeof(path), "/dev/media%d", i);
		int mfd = open(path, O_RDWR);
		if (mfd < 0)
			continue;
		struct media_device_info info;
		struct entity sensor, csi2, ch0, fe;
		if (xioctl(mfd, MEDIA_IOC_DEVICE_INFO, &info) || strcmp(info.driver, "rp1-cfe") ||
		    find_entity(mfd, "vd56g3", &sensor)) {
			close(mfd);
			continue;
		}
		if (find_entity(mfd, "csi2", &csi2) || find_entity(mfd, "rp1-cfe-csi2_ch0", &ch0))
			die("unexpected media graph on %s", path);
		if (!find_entity(mfd, "pisp-fe", &fe))
			setup_link(mfd, csi2.id, 4, fe.id, 0, 0);
		setup_link(mfd, csi2.id, 4, ch0.id, 0, 1);
		set_subdev_fmt(sensor.devnode, 0);
		set_subdev_fmt(csi2.devnode, 0);
		set_subdev_fmt(csi2.devnode, 4);
		snprintf(video, n, "%s", ch0.devnode);
		fprintf(stderr, "vd16gz-bridge: sensor '%s' (%s) on %s, capturing from %s\n",
			sensor.name, sensor.devnode, path, video);

		if (opt.exposure || opt.gain >= 0) {
			int sfd = open(sensor.devnode, O_RDWR);
			if (sfd < 0)
				die("cannot open %s", sensor.devnode);
			if (opt.exposure && !strcmp(opt.exposure, "auto")) {
				set_ctrl(sfd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_AUTO, "auto_exposure");
			} else {
				set_ctrl(sfd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, "auto_exposure");
				if (opt.exposure)
					set_ctrl(sfd, V4L2_CID_EXPOSURE, atoi(opt.exposure), "exposure");
				if (opt.gain >= 0)
					set_ctrl(sfd, V4L2_CID_ANALOGUE_GAIN, opt.gain, "analogue_gain");
			}
			close(sfd);
		}
		close(mfd);
		return;
	}
	errno = 0;
	die("VD16GZ not found. Is the vd56g3 module loaded and 'dtoverlay=pcb4188_vd16gz,cam0' in /boot/firmware/config.txt?");
}

/* ------------------------------------------------------------------ devices */

struct vbuf {
	void *ptr;
	size_t len;
};

struct capture {
	int fd;
	uint32_t stride;
	struct vbuf buf[NCAP];
};

/* v4l2loopback (>= 0.13) tells writers how many applications are streaming from a device. */
#define V4L2LOOPBACK_EVENT_CLIENT_USAGE (V4L2_EVENT_PRIVATE_START + 0x08E00000 + 1)

struct output {
	int fd;
	int readers; /* streaming consumers; -1 if v4l2loopback can not tell (always produce) */
	const char *name;
	uint32_t w, h, stride, size;
	unsigned nbuf, queued;
	struct vbuf buf[NOUT];
	struct v4l2_buffer cur;
};

/* Request and map up to count buffers; returns how many the driver gave (at least 2). */
static unsigned map_buffers(int fd, enum v4l2_buf_type type, struct vbuf *bufs, unsigned count, const char *what)
{
	struct v4l2_requestbuffers rb = { .count = count, .type = type, .memory = V4L2_MEMORY_MMAP };
	if (xioctl(fd, VIDIOC_REQBUFS, &rb) || rb.count < 2)
		die("cannot allocate %s buffers", what);
	if (rb.count > count)
		rb.count = count;
	for (unsigned i = 0; i < rb.count; i++) {
		struct v4l2_buffer b = { .index = i, .type = type, .memory = V4L2_MEMORY_MMAP };
		if (xioctl(fd, VIDIOC_QUERYBUF, &b))
			die("QUERYBUF %s", what);
		bufs[i].len = b.length;
		bufs[i].ptr = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
		if (bufs[i].ptr == MAP_FAILED)
			die("mmap %s", what);
	}
	return rb.count;
}

static void open_capture(struct capture *c, const char *video)
{
	c->fd = open(video, O_RDWR | O_NONBLOCK);
	if (c->fd < 0)
		die("cannot open %s (is another program using the camera?)", video);
	struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
	f.fmt.pix.width = W;
	f.fmt.pix.height = H;
	f.fmt.pix.pixelformat = V4L2_PIX_FMT_Y10P;
	f.fmt.pix.field = V4L2_FIELD_NONE;
	if (xioctl(c->fd, VIDIOC_S_FMT, &f) || f.fmt.pix.pixelformat != V4L2_PIX_FMT_Y10P)
		die("cannot set Y10P on %s", video);
	c->stride = f.fmt.pix.bytesperline;
	map_buffers(c->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE, c->buf, NCAP, "capture");
	for (unsigned i = 0; i < NCAP; i++) {
		struct v4l2_buffer b = { .index = i, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
		if (xioctl(c->fd, VIDIOC_QBUF, &b))
			die("QBUF capture");
	}
	enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(c->fd, VIDIOC_STREAMON, &t))
		die("cannot start streaming on %s", video);
}

static int open_output(struct output *o, const char *path, uint32_t w, uint32_t h, uint32_t fourcc, uint32_t bpp)
{
	if (!path || !strcmp(path, "none")) {
		o->fd = -1;
		return 0;
	}
	o->fd = open(path, O_RDWR);
	if (o->fd < 0)
		die("cannot open %s (is v4l2loopback loaded? see systemd/vd16gz-loopback.conf)", path);
	struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT };
	f.fmt.pix.width = w;
	f.fmt.pix.height = h;
	f.fmt.pix.pixelformat = fourcc;
	f.fmt.pix.field = V4L2_FIELD_NONE;
	f.fmt.pix.bytesperline = w * bpp;
	f.fmt.pix.sizeimage = w * h * bpp;
	f.fmt.pix.colorspace = fourcc == V4L2_PIX_FMT_RGB24 ? V4L2_COLORSPACE_SRGB : V4L2_COLORSPACE_RAW;
	if (xioctl(o->fd, VIDIOC_S_FMT, &f) || f.fmt.pix.pixelformat != fourcc || f.fmt.pix.width != w)
		die("cannot set output format on %s (is another program writing to it?)", path);
	o->w = w;
	o->h = h;
	o->stride = f.fmt.pix.bytesperline;
	o->size = f.fmt.pix.sizeimage;
	struct v4l2_streamparm sp = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT };
	sp.parm.output.timeperframe = (struct v4l2_fract){ 1, 60 };
	xioctl(o->fd, VIDIOC_S_PARM, &sp);
	o->nbuf = map_buffers(o->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT, o->buf, NOUT, "output");
	enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	if (xioctl(o->fd, VIDIOC_STREAMON, &t))
		die("cannot start output on %s", path);
	o->name = path;
	struct v4l2_event_subscription sub = { .type = V4L2LOOPBACK_EVENT_CLIENT_USAGE };
	o->readers = xioctl(o->fd, VIDIOC_SUBSCRIBE_EVENT, &sub) ? -1 : 0;
	return 1;
}

/* Only produce frames for outputs somebody is reading. */
static int output_active(const struct output *o)
{
	return o->fd >= 0 && o->readers != 0;
}

static void read_events(struct output *o)
{
	struct v4l2_event ev;
	do {
		if (xioctl(o->fd, VIDIOC_DQEVENT, &ev))
			return;
		if (ev.type == V4L2LOOPBACK_EVENT_CLIENT_USAGE) {
			memcpy(&o->readers, ev.u.data, sizeof(uint32_t));
			fprintf(stderr, "vd16gz-bridge: %s %s\n", o->name, o->readers ? "in use" : "idle");
		}
	} while (ev.pending);
}

static uint8_t *output_begin(struct output *o)
{
	struct v4l2_buffer *b = &o->cur;
	memset(b, 0, sizeof(*b));
	b->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	b->memory = V4L2_MEMORY_MMAP;
	if (o->queued < o->nbuf) {
		b->index = o->queued++;
	} else if (xioctl(o->fd, VIDIOC_DQBUF, b)) {
		die("DQBUF output");
	}
	return o->buf[b->index].ptr;
}

static void output_end(struct output *o, const struct v4l2_buffer *src)
{
	struct v4l2_buffer *b = &o->cur;
	b->bytesused = o->size;
	b->field = V4L2_FIELD_NONE;
	b->flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	b->timestamp = src->timestamp;
	b->sequence = src->sequence;
	if (xioctl(o->fd, VIDIOC_QBUF, b))
		die("QBUF output");
}

/* -------------------------------------------------------------- processing */

#define LUT_SIZE 4096

#define CHUNK 8 /* rows per demosaic step; keeps the working set in L2 */

static int cfa[4][4];
/* Per row phase (y % 4) and channel, one value per column:
 * mpat = 1 where the pixel has that colour, else 0;
 * ipat = 1 / (sum of kernel weights on that colour's samples), 0 on the colour's own sites. */
static float *mpat[4][4], *ipat[4][4];
static const float K3[] = { 1, 2, 1 }, K5[] = { 1, 2, 3, 2, 1 };
static float lin_lut[1024];
static uint8_t out_lut[LUT_SIZE], ir_lut[LUT_SIZE];

static const float *kernel(int c, int *r)
{
	*r = (c == R || c == B) ? 2 : 1;
	return *r == 2 ? K5 : K3;
}

static void init_tables(void)
{
	for (int y = 0; y < 4; y++)
		for (int x = 0; x < 4; x++) {
			char ch = CFA_ROWS[y][x];
			if (opt.swap_rb && (ch == 'R' || ch == 'B'))
				ch = ch == 'R' ? 'B' : 'R';
			cfa[y][x] = ch == 'R' ? R : ch == 'G' ? G : ch == 'B' ? B : I;
		}
	/* Interpolation is a normalized convolution of each channel's samples
	 * (G and IR with a 3x3 [1 2 1] kernel, R and B with 5x5 [1 2 3 2 1]);
	 * inside the image the normalization only depends on the CFA phase. */
	for (int py = 0; py < 4; py++)
		for (int c = 0; c < 4; c++) {
			mpat[py][c] = malloc(sizeof(float) * W);
			ipat[py][c] = malloc(sizeof(float) * W);
			if (!mpat[py][c] || !ipat[py][c])
				die("out of memory");
			int r;
			const float *k = kernel(c, &r);
			for (int x = 0; x < W; x++) {
				int px = x & 3, own = cfa[py][px] == c;
				float den = 0;
				for (int dy = -r; dy <= r; dy++)
					for (int dx = -r; dx <= r; dx++)
						if (cfa[(py + dy) & 3][(px + dx) & 3] == c)
							den += k[dy + r] * k[dx + r];
				mpat[py][c][x] = own;
				ipat[py][c][x] = own ? 0 : 1 / den;
			}
		}
	for (int v = 0; v < 1024; v++) {
		float l = (float)(v - opt.black) / (1023 - opt.black);
		lin_lut[v] = l < 0 ? 0 : l;
	}
	for (int i = 0; i < LUT_SIZE; i++) {
		float x = (float)i / (LUT_SIZE - 1);
		float s = x <= 0.0031308f ? x * 12.92f : 1.055f * powf(x, 1 / 2.4f) - 0.055f;
		out_lut[i] = (uint8_t)((opt.linear ? x : s) * 255 + 0.5f);
		ir_lut[i] = out_lut[i];
	}
}

static inline uint8_t to8(const uint8_t *lut, float v)
{
	int i = (int)(v + 0.5f); /* v is pre-scaled to 0..LUT_SIZE-1 */
	return lut[i < 0 ? 0 : i >= LUT_SIZE ? LUT_SIZE - 1 : i];
}

static inline int reflect(int i, int n)
{
	if (i < 0)
		i = -i;
	if (i >= n)
		i = 2 * n - 2 - i;
	return i;
}

/* Border pixels: same normalized convolution, with the image mirrored at its edges.
 * In the corners a channel may have no sample in reach; widen the kernel there. */
static float interp_border(const float *lin, int y, int x, int c)
{
	int r;
	const float *k = kernel(c, &r);
	float wide[9];
	for (int pass = 0; pass < 2; pass++) {
		float num = 0, den = 0;
		for (int dy = -r; dy <= r; dy++)
			for (int dx = -r; dx <= r; dx++) {
				int yy = reflect(y + dy, H), xx = reflect(x + dx, W);
				if (cfa[yy & 3][xx & 3] != c)
					continue;
				float w = k[dy + r] * k[dx + r];
				num += w * lin[yy * W + xx];
				den += w;
			}
		if (den > 0)
			return num / den;
		/* wide kernel = k convolved with itself */
		int n = 2 * r + 1;
		memset(wide, 0, sizeof(wide));
		for (int a = 0; a < n; a++)
			for (int b = 0; b < n; b++)
				wide[a + b] += k[a] * k[b];
		k = wide;
		r *= 2;
	}
	return 0;
}

struct frame_ctx {
	const uint8_t *raw;
	uint8_t *staging;
	uint32_t raw_stride;
	float *lin;
	uint16_t *raw16;  /* may be NULL */
	uint8_t *rgb, *ir; /* may be NULL */
	uint32_t rgb_stride, ir_stride;
	float gain[3];
	double stats[MAX_THREADS][4];
	float *scratch[MAX_THREADS]; /* demosaic working rows per thread */
};

static struct frame_ctx fc;

/* Phase 1: unpack Y10P rows into linear floats (and the raw 16-bit copy).
 * Capture buffers are uncached, so each band is first copied in bulk to normal memory. */
static void unpack_rows(int y0, int y1, int tid)
{
	(void)tid;
	uint8_t *staging = fc.staging + (size_t)y0 * fc.raw_stride;
	memcpy(staging, fc.raw + (size_t)y0 * fc.raw_stride, (size_t)(y1 - y0) * fc.raw_stride);
	for (int y = y0; y < y1; y++) {
		const uint8_t *s = staging + (size_t)(y - y0) * fc.raw_stride;
		float *l = fc.lin + (size_t)y * W;
		uint16_t *r16 = fc.raw16 ? fc.raw16 + (size_t)y * W : NULL;
		for (int x = 0; x < W; x += 4, s += 5) {
			for (int i = 0; i < 4; i++) {
				uint16_t v = (s[i] << 2) | ((s[4] >> (2 * i)) & 3);
				l[x + i] = lin_lut[v];
				if (r16)
					r16[x + i] = v;
			}
		}
	}
}

/* Shared colour path for one output row: IR subtraction, colour matrix, white balance,
 * 8-bit conversion. Settings are copied to locals so they stay in registers. */
static void emit_row(int tid, int y, int n, const float *restrict pr, const float *restrict pg,
		     const float *restrict pb, const float *restrict pi)
{
	const float sub = opt.ir_subtract;
	const int have_ccm = opt.have_ccm, stat_row = !(y & 7);
	float M[3][4];
	memcpy(M, opt.ccm, sizeof(M));
	const float scale = LUT_SIZE - 1;
	const float g0 = fc.gain[0] * scale, g1 = fc.gain[1] * scale, g2 = fc.gain[2] * scale;
	uint8_t *restrict rgb = fc.rgb ? fc.rgb + (size_t)y * fc.rgb_stride : NULL;
	uint8_t *restrict irp = fc.ir ? fc.ir + (size_t)y * fc.ir_stride : NULL;
	if (!rgb) { /* IR only: R, G and B were not computed */
		for (int x = 0; irp && x < n; x++)
			irp[x] = to8(ir_lut, pi[x] * scale);
		return;
	}
	double s0 = 0, s1 = 0, s2 = 0, cnt = 0;
	for (int x = 0; x < n; x++) {
		float r = pr[x], g = pg[x], b = pb[x], ir = pi[x];
		if (sub > 0) {
			r = fmaxf(r - sub * ir, 0);
			g = fmaxf(g - sub * ir, 0);
			b = fmaxf(b - sub * ir, 0);
		}
		if (have_ccm) {
			float r2 = M[0][0] * r + M[0][1] * g + M[0][2] * b + M[0][3] * ir;
			float g2 = M[1][0] * r + M[1][1] * g + M[1][2] * b + M[1][3] * ir;
			float b2 = M[2][0] * r + M[2][1] * g + M[2][2] * b + M[2][3] * ir;
			r = r2, g = g2, b = b2;
		}
		if (stat_row && !(x & 7))
			s0 += r, s1 += g, s2 += b, cnt += 1;
		rgb[3 * x] = to8(out_lut, r * g0);
		rgb[3 * x + 1] = to8(out_lut, g * g1);
		rgb[3 * x + 2] = to8(out_lut, b * g2);
		if (irp)
			irp[x] = to8(ir_lut, ir * scale);
	}
	double *s = fc.stats[tid];
	s[0] += s0, s[1] += s1, s[2] += s2, s[3] += cnt;
}

/* Phase 2 (full resolution): interpolate the missing channels at every pixel. */

#define HROWS (CHUNK + 4)
#define SCRATCH_FLOATS ((size_t)4 * (HROWS + CHUNK) * W)

/* All four channels of an edge pixel, written into the row planes p[R..I]. */
static void border_pixel(float *const *p, int y, int x)
{
	for (int c = 0; c < 4; c++)
		p[c][x] = cfa[y & 3][x & 3] == c ? fc.lin[(size_t)y * W + x] : interp_border(fc.lin, y, x, c);
}

/* Horizontal pass over one source row of one channel: out = sum k[d] * (lin * mask)[x + d].
 * Written with a constant radius so the compiler unrolls and vectorizes it. */
static inline __attribute__((always_inline)) void hpass(const float *l, const float *m, float *out,
							 const float *k, int r)
{
	for (int x = 2; x < W - 2; x++) {
		float s = 0;
		for (int d = -r; d <= r; d++)
			s += k[d + r] * l[x + d] * m[x + d];
		out[x] = s;
	}
}

/* Vertical pass and normalization for one output row of one channel. */
static inline __attribute__((always_inline)) void vpass(float *const *h, const float *l, const float *m,
							 const float *ip, float *out, const float *k, int r)
{
	for (int x = 2; x < W - 2; x++) {
		float s = 0;
		for (int d = -r; d <= r; d++)
			s += k[d + r] * h[d + r][x];
		out[x] = s * ip[x] + l[x] * m[x]; /* measured pixels are kept exact */
	}
}

static void demosaic_chunk(int a, int b, int tid)
{
	/* scratch: hrows[c][i] holds source row a - 2 + i after the horizontal pass;
	 * plane[c][j] holds output row a + j. */
	float *hrows = fc.scratch[tid], *plane = hrows + (size_t)4 * HROWS * W;
	for (int c = 0; c < 4; c++) {
		if (!fc.rgb && c != I)
			continue;
		int r;
		kernel(c, &r);
		float *hc = hrows + (size_t)c * HROWS * W;
		for (int y = a - r; y < b + r; y++) {
			const float *l = fc.lin + (size_t)y * W, *m = mpat[y & 3][c];
			float *o = hc + (size_t)(y - a + 2) * W;
			if (r == 2)
				hpass(l, m, o, K5, 2);
			else
				hpass(l, m, o, K3, 1);
		}
		for (int y = a; y < b; y++) {
			float *h[5];
			for (int d = -r; d <= r; d++)
				h[d + r] = hc + (size_t)(y + d - a + 2) * W;
			float *o = plane + ((size_t)c * CHUNK + (y - a)) * W;
			const float *l = fc.lin + (size_t)y * W;
			if (r == 2)
				vpass(h, l, mpat[y & 3][c], ipat[y & 3][c], o, K5, 2);
			else
				vpass(h, l, mpat[y & 3][c], ipat[y & 3][c], o, K3, 1);
		}
	}
	for (int y = a; y < b; y++) {
		float *p[4];
		for (int c = 0; c < 4; c++)
			p[c] = plane + ((size_t)c * CHUNK + (y - a)) * W;
		for (int x = 0; x < 2; x++) {
			border_pixel(p, y, x);
			border_pixel(p, y, W - 1 - x);
		}
		emit_row(tid, y, W, p[R], p[G], p[B], p[I]);
	}
}

static void demosaic_rows(int y0, int y1, int tid)
{
	for (int y = y0; y < y1; y++) {
		if (y >= 2 && y < H - 2) {
			int b = y + CHUNK;
			if (b > y1)
				b = y1;
			if (b > H - 2)
				b = H - 2;
			demosaic_chunk(y, b, tid);
			y = b - 1;
			continue;
		}
		float *p[4];
		for (int c = 0; c < 4; c++)
			p[c] = fc.scratch[tid] + (size_t)c * W;
		for (int x = 0; x < W; x++)
			border_pixel(p, y, x);
		emit_row(tid, y, W, p[R], p[G], p[B], p[I]);
	}
}

/* Phase 2 (half resolution): each 2x2 block holds G, IR, G and one R-or-B pixel;
 * the missing one of R/B is the mean of the 4 neighbouring blocks.
 * The 562 block columns are cropped to 560 so every output row is a multiple of
 * 4 bytes long: OpenCV assumes 4-byte aligned rows and shears the image otherwise. */
#define HALF_W 560
#define HALF_X0 1 /* first block column used */

static inline float half_c(const float *lin, int y, int x)
{
	y = reflect(y, H / 2), x = reflect(x, W / 2);
	return lin[(size_t)(2 * y + 1) * W + 2 * x];
}

static void half_rows(int y0, int y1, int tid)
{
	const float *lin = fc.lin;
	float *pr = fc.scratch[tid], *pg = pr + W, *pb = pg + W, *pi = pb + W;
	for (int y = y0; y < y1; y++) {
		for (int x = 0; x < HALF_W; x++) {
			int bx = x + HALF_X0;
			const float *p = lin + (size_t)(2 * y) * W + 2 * bx;
			float c = p[W];
			float other = (half_c(lin, y - 1, bx) + half_c(lin, y + 1, bx) +
				       half_c(lin, y, bx - 1) + half_c(lin, y, bx + 1)) * 0.25f;
			/* cfa row 1 starts with B (or R with --swap-rb), so even (bx+y) holds that colour */
			int c_is_r = ((bx + y) & 1) ^ (cfa[1][0] == R);
			pr[x] = c_is_r ? c : other;
			pb[x] = c_is_r ? other : c;
			pg[x] = (p[0] + p[W + 1]) * 0.5f;
			pi[x] = p[1];
		}
		emit_row(tid, y, HALF_W, pr, pg, pb, pi);
	}
}

typedef void (*row_fn)(int, int, int);

struct job {
	row_fn fn;
	int y0, y1, tid;
};

static void *job_thread(void *arg)
{
	struct job *j = arg;
	j->fn(j->y0, j->y1, j->tid);
	return NULL;
}

static void parallel_rows(row_fn fn, int rows)
{
	pthread_t th[MAX_THREADS];
	struct job jobs[MAX_THREADS];
	int n = opt.threads;
	for (int i = 0; i < n; i++) {
		jobs[i] = (struct job){ fn, rows * i / n, rows * (i + 1) / n, i };
		if (i && pthread_create(&th[i], NULL, job_thread, &jobs[i]))
			die("pthread_create");
	}
	job_thread(&jobs[0]);
	for (int i = 1; i < n; i++)
		pthread_join(th[i], NULL);
}

/* ------------------------------------------------------------------ config */

static void load_ccm(const char *path, int required)
{
	FILE *f = fopen(path, "r");
	if (!f) {
		if (required)
			die("cannot open colour matrix %s", path);
		return;
	}
	char buf[8192];
	size_t n = fread(buf, 1, sizeof(buf) - 1, f);
	fclose(f);
	buf[n] = 0;
	/* ccm.json from tools/calibrate.py: {"matrix": [[r0..r3], [g0..g3], [b0..b3]], ...} */
	char *p = strstr(buf, "\"matrix\"");
	if (!p)
		die("no \"matrix\" in %s", path);
	p += 8;
	for (int i = 0; i < 12; i++) {
		while (*p && !(*p == '-' || *p == '.' || (*p >= '0' && *p <= '9')))
			p++;
		char *end;
		opt.ccm[i / 4][i % 4] = strtof(p, &end);
		if (end == p)
			die("colour matrix in %s must be 3x4", path);
		p = end;
	}
	opt.have_ccm = 1;
	fprintf(stderr, "vd16gz-bridge: using colour matrix %s\n", path);
}

static void usage(void)
{
	printf("Usage: vd16gz-bridge [options]\n"
	       "Split the VD16GZ RGB-IR mosaic into RGB, IR and raw V4L2 loopback devices.\n\n"
	       "  --rgb DEV          RGB24 output   (default /dev/vd16gz-rgb, 'none' to disable)\n"
	       "  --ir DEV           GREY output    (default /dev/vd16gz-ir, 'none' to disable)\n"
	       "  --raw DEV          Y16 raw mosaic (default /dev/vd16gz-raw, 'none' to disable)\n"
	       "  --half             560x682 output (2x2 binned demosaic, less CPU)\n"
	       "  --linear           linear 8-bit output instead of sRGB gamma\n"
	       "  --wb MODE          'auto' (grey world), 'none' or R,G,B gains (default: auto,\n"
	       "                     or none when a colour matrix is loaded)\n"
	       "  --ccm FILE         3x4 colour matrix from tools/calibrate.py\n"
	       "                     (default /etc/vd16gz/ccm.json if it exists)\n"
	       "  --ir-subtract K    rgb -= K * ir before the colour matrix\n"
	       "  --swap-rb          swap the R and B sites of the colour filter\n"
	       "  --black N          black level of the 10-bit data (default 64)\n"
	       "  --exposure auto|N  set sensor auto exposure or N lines (21-2093) at start\n"
	       "  --gain N           set sensor analogue gain (0-28) at start\n"
	       "  --threads N        worker threads (default 4)\n"
	       "  -v, --verbose      print frame rate statistics every 10 s\n");
}

static void parse_args(int argc, char **argv)
{
	static const struct option lo[] = {
		{ "rgb", 1, 0, 'r' }, { "ir", 1, 0, 'i' }, { "raw", 1, 0, 'R' },
		{ "half", 0, 0, 'h' }, { "linear", 0, 0, 'l' }, { "wb", 1, 0, 'w' },
		{ "ccm", 1, 0, 'c' }, { "ir-subtract", 1, 0, 's' }, { "swap-rb", 0, 0, 'S' },
		{ "black", 1, 0, 'b' }, { "exposure", 1, 0, 'e' }, { "gain", 1, 0, 'g' },
		{ "threads", 1, 0, 't' }, { "verbose", 0, 0, 'v' }, { "help", 0, 0, 'H' },
		{ 0 },
	};
	const char *ccm = NULL;
	int c;
	while ((c = getopt_long(argc, argv, "v", lo, NULL)) != -1) {
		switch (c) {
		case 'r': opt.dev_path[0] = optarg; break;
		case 'i': opt.dev_path[1] = optarg; break;
		case 'R': opt.dev_path[2] = optarg; break;
		case 'h': opt.half = 1; break;
		case 'l': opt.linear = 1; break;
		case 'w':
			if (!strcmp(optarg, "auto"))
				opt.wb_mode = 1;
			else if (!strcmp(optarg, "none"))
				opt.wb_mode = 0;
			else if (sscanf(optarg, "%f,%f,%f", &opt.wb_fixed[0], &opt.wb_fixed[1], &opt.wb_fixed[2]) == 3)
				opt.wb_mode = 2;
			else
				errno = 0, die("--wb takes auto, none or R,G,B");
			break;
		case 'c': ccm = optarg; break;
		case 's': opt.ir_subtract = strtof(optarg, NULL); break;
		case 'S': opt.swap_rb = 1; break;
		case 'b': opt.black = atoi(optarg); break;
		case 'e': opt.exposure = optarg; break;
		case 'g': opt.gain = atoi(optarg); break;
		case 't': opt.threads = atoi(optarg); break;
		case 'v': opt.verbose = 1; break;
		case 'H': usage(); exit(0);
		default: usage(); exit(2);
		}
	}
	if (opt.threads < 1 || opt.threads > MAX_THREADS)
		errno = 0, die("--threads must be 1..%d", MAX_THREADS);
	if (opt.black < 0 || opt.black > 1000)
		errno = 0, die("--black out of range");
	if (ccm && strcmp(ccm, "none"))
		load_ccm(ccm, 1);
	else if (!ccm)
		load_ccm("/etc/vd16gz/ccm.json", 0);
	if (opt.wb_mode < 0)
		opt.wb_mode = opt.have_ccm ? 0 : 1;
}

/* -------------------------------------------------------------------- main */

static void on_signal(int s)
{
	(void)s;
	stop = 1;
}

static double now(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec * 1e-9;
}

int main(int argc, char **argv)
{
	parse_args(argc, argv);
	init_tables();
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	char video[128];
	setup_pipeline(video, sizeof(video));

	int ow = opt.half ? HALF_W : W, oh = opt.half ? H / 2 : H;
	struct output out[3];
	int any = 0;
	any |= open_output(&out[0], opt.dev_path[0], ow, oh, V4L2_PIX_FMT_RGB24, 3);
	any |= open_output(&out[1], opt.dev_path[1], ow, oh, V4L2_PIX_FMT_GREY, 1);
	any |= open_output(&out[2], opt.dev_path[2], W, H, V4L2_PIX_FMT_Y16, 2);
	if (!any)
		errno = 0, die("all outputs disabled");

	struct capture cap;
	open_capture(&cap, video);

	fc.lin = malloc(sizeof(float) * W * H);
	fc.staging = malloc((size_t)cap.stride * H);
	if (!fc.lin || !fc.staging)
		die("out of memory");
	for (int t = 0; t < opt.threads; t++)
		if (!(fc.scratch[t] = malloc(sizeof(float) * SCRATCH_FLOATS)))
			die("out of memory");
	float wb[3] = { 1, 1, 1 };
	if (opt.wb_mode == 2)
		memcpy(wb, opt.wb_fixed, sizeof(wb));
	fprintf(stderr, "vd16gz-bridge: streaming %dx%d %s, %s, wb %s\n", ow, oh,
		opt.half ? "(half)" : "(full)", opt.linear ? "linear" : "sRGB",
		opt.wb_mode == 1 ? "auto" : opt.wb_mode == 2 ? "fixed" : "none");

	unsigned long frames = 0, dropped = 0;
	double t_stat = now(), busy = 0;
	uint32_t last_seq = 0;
	int have_seq = 0;
	while (!stop) {
		struct pollfd pfd[4] = { { .fd = cap.fd, .events = POLLIN } };
		for (int i = 0; i < 3; i++)
			pfd[i + 1] = (struct pollfd){ .fd = out[i].readers >= 0 ? out[i].fd : -1, .events = POLLPRI };
		int pr = poll(pfd, 4, 2000);
		if (pr < 0) {
			if (errno == EINTR)
				continue;
			die("poll");
		}
		if (pr == 0) {
			fprintf(stderr, "vd16gz-bridge: no frames from sensor for 2 s\n");
			continue;
		}
		for (int i = 0; i < 3; i++)
			if (pfd[i + 1].revents & POLLPRI)
				read_events(&out[i]);
		if (!(pfd[0].revents & POLLIN))
			continue;
		/* Take the newest frame; if we fell behind, give older ones straight back. */
		struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
		if (xioctl(cap.fd, VIDIOC_DQBUF, &b)) {
			if (errno == EAGAIN)
				continue;
			die("DQBUF capture");
		}
		for (;;) {
			struct v4l2_buffer nb = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
			if (xioctl(cap.fd, VIDIOC_DQBUF, &nb))
				break;
			xioctl(cap.fd, VIDIOC_QBUF, &b);
			b = nb;
		}
		if (have_seq && b.sequence != last_seq + 1)
			dropped += b.sequence - last_seq - 1;
		last_seq = b.sequence, have_seq = 1;
		int active[3];
		for (int i = 0; i < 3; i++)
			active[i] = output_active(&out[i]);
		if (!active[0] && !active[1] && !active[2]) {
			xioctl(cap.fd, VIDIOC_QBUF, &b); /* nobody watching: keep the sensor running, skip the work */
			continue;
		}
		double t0 = now();

		fc.raw = cap.buf[b.index].ptr;
		fc.raw_stride = cap.stride;
		fc.raw16 = active[2] ? (uint16_t *)output_begin(&out[2]) : NULL;
		fc.rgb = active[0] ? output_begin(&out[0]) : NULL;
		fc.ir = active[1] ? output_begin(&out[1]) : NULL;
		fc.rgb_stride = out[0].stride;
		fc.ir_stride = out[1].stride;
		memcpy(fc.gain, wb, sizeof(wb));
		memset(fc.stats, 0, sizeof(fc.stats));

		parallel_rows(unpack_rows, H);
		struct v4l2_buffer frame = b; /* QBUF clears the timestamp, keep it for the outputs */
		xioctl(cap.fd, VIDIOC_QBUF, &b); /* raw data no longer needed */
		if (fc.rgb || fc.ir)
			parallel_rows(opt.half ? half_rows : demosaic_rows, oh);

		for (int i = 0; i < 3; i++)
			if (active[i])
				output_end(&out[i], &frame);

		if (opt.wb_mode == 1 && fc.rgb) { /* smoothed grey world for the next frame */
			double s[3] = { 0 };
			for (int t = 0; t < opt.threads; t++)
				for (int c = 0; c < 3; c++)
					s[c] += fc.stats[t][c];
			for (int c = 0; c < 3; c++)
				wb[c] = 0.9f * wb[c] + 0.1f * (float)(s[1] / fmax(s[c], 1e-6));
		}

		frames++;
		busy += now() - t0;
		if (opt.verbose && now() - t_stat >= 10) {
			double dt = now() - t_stat;
			fprintf(stderr, "vd16gz-bridge: %.1f fps, %.1f ms/frame processing, %lu sensor frames dropped\n",
				frames / dt, 1000 * busy / frames, dropped);
			frames = dropped = 0, busy = 0, t_stat = now();
		}
	}
	fprintf(stderr, "vd16gz-bridge: stopped\n");
	return 0;
}
