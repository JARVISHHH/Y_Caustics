# path-Hebella

## How To Run
To Specify the scene discription file and the output file, please use command line arguments.

Global parameters that can be modified in main.cpp including:
* usePhotonMapping: if true, use photon mapping to render. Else use path tracing method.
* samplePerPixel: number of samples within one pixel.
* defocusBlurOn: whether to apply depth of field. The default camera lens radius is 0.5 and focus length is 3.5.
* useOrenNayerBRDF: whether to use OrenNayer BRDF when rendering diffuse surfaces. If false then use Lambertian.
* importanceSampling: whether to use cosine importance sampling when generating ray directions on the hemisphere. Will be applied to Lambertian, OrenNayer and Glossy surfaces.

## Diffuse
Original cornell box, samples per pixel = 300

![Cornell Box Original](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-original-300spp.png)

## Glossy
Glossy cornell box, samples per pixel = 500

![Cornell Box Glossy 1](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-glossy-500spp.png)

Glossy cornell box, another way of tone mapping, samples per pixel = 300

![Cornell Box Glossy 2](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-glossy-tonemapping.png)

## Mirror
Mirror cornell box, sample per pixel = 1000

![Cornell Box Mirror](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-mirror-1000spp.png)

## Reflection and Refraction
Sphere cornell box, sample per pixel = 5000

![Cornell Box Sphere](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-sphere-5000spp.png)

## Direct lighting
Original cornell box without direct lighting, sample per pixel = 300

![Indirect Lighting](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-original-indirect.png)

## Depth of Field
Original cornell box, sample per pixel = 300, lens radius = 0.5, focus length = 3.7

![Defocus Blur 1](https://github.com/brown-cs-224/path-Hebella/blob/master/output/cornell-0.5-3.7-300spp.png)

Original cornell box, sample per pixel = 300, lens radius = 1.0, focus length = 4.0

![Defocus Blur 1](https://github.com/brown-cs-224/path-Hebella/blob/master/output/cornell-1.0-4.0-300spp.png)

## OreyNayer BDRF
Original cornell box, sample per pixel = 300, $\sigma$ = 0.1

![OreyNayer 1](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-oney-0.1-300spp.png)

Original cornell box, sample per pixel = 300, $\sigma$ = 1.0

![OreyNayer 2](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-oney-1.0-300spp.png)

## Stratified Sampling
I applited stratified sampling to generating random points within pixels.

Mirror cornell box without stratified sampling, sample per pixel = 300

![Stratifies Sampling 1](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-mirror-300spp.png)

Mirror cornell box with stratified sampling, sample per pixel = 300 (fewer noises)

![Stratifies Sampling 2](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-mirror-300spp-stratified.png)

## Importance Sampling
Mirror cornell box with cosine importance sampling, sample per pixel = 300

![Stratifies Sampling 2](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-mirror-300spp-cosine.png)

Mirror cornell box without importance sampling, sample per pixel = 300

![Stratifies Sampling 1](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-mirror-300spp.png)

## Photon Mapping
I implemented photon map in Kd-tree. When setting usePhotonMap to true, scenes will be rendered using photon mapping and direct lighting. I used a photon map of size 50000
for global illumination and another photon map of size 50000 to render caustic effect.

Photon mapping sphere cornell box, sample per pixel = 300

![Photon Mapping](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-sphere-300spp-photon.png)

Path Tracing sphere cornell box, sample per pixel = 300

![Cornell Box Sphere](https://github.com/brown-cs-224/path-Hebella/blob/master/output/result/cornell-sphere-300spp.png)
