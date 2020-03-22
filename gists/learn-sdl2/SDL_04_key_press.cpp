//Using SDL and standard IO
#include <SDL.h>
#include <stdio.h>
#include <string>

enum qxf_enum_SurfaceIndex {
  SurfaceIndexDefault,
  SurfaceIndexUp,
  SurfaceIndexDown,
  SurfaceIndexRight,
  SurfaceIndexLeft,
  SurfaceIndexLimit,
};

const int qxf_c_ScreenWidth = 640;
const int qxf_c_ScreenHeight = 480;

const std::string qxf_c_BmpPaths[ SurfaceIndexLimit ] = {
  "tmp.bmp",
  "testup.bmp",
  "testdown.bmp",
  "testright.bmp",
  "testleft.bmp",
};

bool qxf_Init();
bool qxf_LoadMedia();
bool qxf_Close();

SDL_Surface* qxf_LoadSurface( std::string path );

//The window we'll be rendering to
SDL_Window* gWindow = NULL;
    
//The surface contained by the window
SDL_Surface* gScreenSurface = NULL;

//The image we will load and show on the screen
SDL_Surface* qxf_g_CurrentSurface = NULL;
SDL_Surface* qxf_g_PreloadedSurfaces[ SurfaceIndexLimit ];

bool qxf_Init()
{
  bool success = true;

  //Initialize SDL
  if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
  {
    printf( "SDL could not initialize! SDL_Error: %s\n", SDL_GetError() );
    success = false;
  }
  else
  {
    //Create window
    gWindow = SDL_CreateWindow(
      "SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
      qxf_c_ScreenWidth, qxf_c_ScreenHeight, SDL_WINDOW_SHOWN
    );
    if( gWindow == NULL )
    {
      printf( "Window could not be created! SDL_Error: %s\n", SDL_GetError() );
      success = false;
    }
    else
    {
      //Get window surface
      gScreenSurface = SDL_GetWindowSurface( gWindow );
    }
  }

  return success;
}

bool qxf_LoadMedia()
{
  bool success = true;

  for ( int i = 0; i < SurfaceIndexLimit; ++i )
  {
    qxf_g_PreloadedSurfaces[ i ] = qxf_LoadSurface
    (
      qxf_c_BmpPaths[ i ].c_str()
    );
    if( qxf_g_PreloadedSurfaces[ i ] == NULL )
    {
      printf(
        "Unable to load image %s! SDL Error: %s\n",
        qxf_c_BmpPaths[ i ].c_str(), SDL_GetError()
      );
      success = false;
    }
  }

  return success;
}

bool qxf_Close()
{
  //Deallocate surface
  for ( int i = 0; i < SurfaceIndexLimit; ++i ) {
    SDL_FreeSurface( qxf_g_PreloadedSurfaces[ i ] );
    qxf_g_PreloadedSurfaces[ i ] = NULL;
  }

  //Destroy window
  SDL_DestroyWindow( gWindow );
  gWindow = NULL;

  //Quit SDL subsystems
  SDL_Quit();
}

SDL_Surface* qxf_LoadSurface( std::string path )
{
    //Load image at specified path
    SDL_Surface* loadedSurface = SDL_LoadBMP( path.c_str() );
    if( loadedSurface == NULL )
    {
      printf( "Unable to load image %s! SDL Error: %s\n", path.c_str(), SDL_GetError() );
    }

    return loadedSurface;
}

int main( int argc, char* args[] )
{
	//Start up SDL and create window
  if( false == qxf_Init() )
  {
    printf( "Failed to initialize!\n" );
  }
  else
  {
    //Load media
    if( false == qxf_LoadMedia() )
    {
      printf( "Failed to load media!\n" );
    }
    else
    {
      //Main loop flag
      bool quit = false;
      qxf_g_CurrentSurface = qxf_g_PreloadedSurfaces[ SurfaceIndexDefault ];

      //Event handler
      SDL_Event e;
      
      while ( false == quit )
      {
        while( SDL_PollEvent( &e ) != 0 )
        {
          //User requests quit
          if( e.type == SDL_QUIT )
          {
            quit = true;
          } else if ( e.type == SDL_KEYDOWN )
          {
            switch ( e.key.keysym.sym )
            {
            case SDLK_UP:
              qxf_g_CurrentSurface = qxf_g_PreloadedSurfaces[ SurfaceIndexUp ];
              break;
            case SDLK_DOWN:
              qxf_g_CurrentSurface = qxf_g_PreloadedSurfaces[ SurfaceIndexDown ];
              break;
            case SDLK_RIGHT:
              qxf_g_CurrentSurface = qxf_g_PreloadedSurfaces[ SurfaceIndexRight ];
              break;
            case SDLK_LEFT:
              qxf_g_CurrentSurface = qxf_g_PreloadedSurfaces[ SurfaceIndexLeft ];
              break;
            default:
              qxf_g_CurrentSurface = qxf_g_PreloadedSurfaces[ SurfaceIndexDefault ];
            }
          }
        }
        //Apply the image
        SDL_BlitSurface( qxf_g_CurrentSurface, NULL, gScreenSurface, NULL );
        //Update the surface
        SDL_UpdateWindowSurface( gWindow ); 
      }
    }
  }
  
  //Free resources and close SDL
  qxf_Close();
	return 0;
}