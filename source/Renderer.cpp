//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include <iostream>
#include <thread>
#include <future> // All your async stuff
#include <ppl.h>
 
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"


using namespace dae;

// #define ASYNC
#define PARALLEL_FOR

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
	m_AspectRatio = static_cast<float>(m_Width) / static_cast<float>(m_Height);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	const uint32_t numPixels = m_Width * m_Height;

#if defined(ASYNC)
	// Async Logic
	const uint32_t numCores = std::thread::hardware_concurrency();
	std::vector<std::future<void>> async_futures{};

	const uint32_t numPixelsPerTask = numPixels / numCores;
	uint32_t numUnassignedPixel = numPixels % numCores;
	uint32_t currPixelIndex{ 0 };

	for (uint32_t coreId{ 0 }; coreId < numCores; ++coreId)
	{
		uint32_t taskSize{ numPixelsPerTask };
		if (numUnassignedPixel > 0)
		{
			++taskSize;
			--numUnassignedPixel;
		}

		async_futures.emplace_back(
			std::async(std::launch::async, [=, this]
				{
					const uint32_t pixelIndexEnd = currPixelIndex + taskSize;
					for (uint32_t pixelIndex{ currPixelIndex }; pixelIndex < pixelIndexEnd; ++pixelIndex)
					{
						RenderPixel(pScene, pixelIndex, camera, lights, materials);
					}
				})
		);

		currPixelIndex += taskSize;
	}

	// Wait for all tasks
	for (const std::future<void>& f : async_futures)
	{
		f.wait();
	}

#elif defined(PARALLEL_FOR)
	concurrency::parallel_for(0u, numPixels,
		[=, this](int i)
		{
			RenderPixel(pScene, i, camera, lights, materials);
		});
#else
	// Snychronous Logic
	for (uint32_t i = 0; i < numPixels; i++)
	{
		RenderPixel(pScene, i, camera, lights, materials);
	}
#endif

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

void dae::Renderer::RenderPixel(Scene* pScene, uint32_t pixelIndex, const Camera& camera, const std::vector<Light>& lights, const std::vector<Material*>& materials) const
{
	const int px = static_cast<int>(pixelIndex % m_Width + 0.5f);
	const int py = static_cast<int>(pixelIndex / m_Width + 0.5f);

	const float cx{ ((2.f * px / static_cast<float>(m_Width)) - 1.f) * m_AspectRatio * camera.fov };
	const float cy{ (1.f - (2.f * py / static_cast<float>(m_Height))) * camera.fov };

	Vector3 rayDirection = { cx, cy, 1.f };
	rayDirection = camera.cameraToWorld.TransformVector(rayDirection);

	Ray viewRay{ camera.origin, rayDirection };
	ColorRGB finalColor{};

	HitRecord closestHit{};
	pScene->GetClosestHit(viewRay, closestHit);

	if (closestHit.didHit)
	{
		for (const Light& light : lights)
		{
			Ray lightRay{};
			lightRay.origin = closestHit.origin;
			lightRay.direction = LightUtils::GetDirectionToLight(light, lightRay.origin + closestHit.normal * 0.01f);
			lightRay.min = 0.1f;
			lightRay.max = lightRay.direction.Magnitude();
			lightRay.direction.Normalize();

			if ((m_ShadowsEnabled && pScene->DoesHit(lightRay))) continue;

			const float dpObservedArea{ Vector3::Dot(closestHit.normal, lightRay.direction) };

			switch (m_CurrentLightingMode)
			{
			case dae::Renderer::LightingMode::ObservedArea:
				if (dpObservedArea < 0) continue;
				finalColor += { dpObservedArea, dpObservedArea, dpObservedArea };
				break;
			case dae::Renderer::LightingMode::Radiance:
				finalColor += LightUtils::GetRadiance(light, lightRay.origin);
				break;
			case dae::Renderer::LightingMode::BRDF:
				finalColor += materials[closestHit.materialIndex]->Shade(closestHit, lightRay.direction, viewRay.direction);
				break;
			case dae::Renderer::LightingMode::Combined:
				if (dpObservedArea < 0) continue;
				finalColor += LightUtils::GetRadiance(light, lightRay.origin) * materials[closestHit.materialIndex]->Shade(closestHit, lightRay.direction, viewRay.direction) * dpObservedArea;
				break;
			default:
				std::cout << "Error getting lighting mode!\n";
				break;
			}
		}
	}

	//Update Color in Buffer
	finalColor.MaxToOne();

	m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode()
{
	m_CurrentLightingMode = static_cast<LightingMode>(
		(static_cast<int>(m_CurrentLightingMode) + 1) % 4);
}
