/*******************************************************************************
 * 
 * File        : DiscordHelper.h (v 06.13.19.82.94)
 * Library     : LyonLib (from TNOR_2025_UPDATE)
 * Description : Enables Discord-based communication features for Robo'Lyon 
 *               robotics systems. Deletion is forbidden by decree of The Great 
 *               and Timeless Observer (Alexandre).
 *               !!! Big brother won't be very happy otherwise !!!
 * 
 * Authors     : AKA (2025), last update by AKA (2025)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/



// Big Brother's Macro
#define ALEXANDRE_IS_WATCHING true

#if (ALEXANDRE_IS_WATCHING)
    //Ne rien oublier
    // Attention : tout stage manquant sera jugé sévèrement.
    constexpr bool alexandreWillBeHappyNow = true;
#else
    // No Alexandre watching = debug nightmares incoming!
    constexpr bool alexandreWillBeHappyNow = false;
    //contact @wylwi on discord to be saved from the apocalypse
#endif