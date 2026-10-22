#pragma once
#include "ofMain.h"
#include "imgui.h"

// The look shared by the Echo & Narcissus tools (femtoSkeletonOsc, oscRecPlay, echoNarcissus): one dark, dense,
// quiet operator console, where a colour always means the same thing. See SkeletonTracking/DESIGN.md.
// The same file is in SkeletonTracking/femtoSkeletonOsc/src, SkeletonTracking/oscRecPlay/src and echoNarcissus/src.
namespace optheme {

inline ofColor hex( uint32_t aRgb, int aAlpha = 255 ) { return ofColor(( aRgb >> 16 ) & 255, ( aRgb >> 8 ) & 255, aRgb & 255, aAlpha ); }
inline ImVec4 im( const ofColor& c, float aAlpha = -1.f ) { return ImVec4( c.r / 255.f, c.g / 255.f, c.b / 255.f, aAlpha >= 0.f ? aAlpha : c.a / 255.f ); }

// --- surfaces (slate) ---
inline const ofColor bg         = hex( 0x0F172A );   // the window behind everything, 3D view backgrounds
inline const ofColor panel      = hex( 0x1B2336 );   // control windows
inline const ofColor raised     = hex( 0x272F42 );   // fields, headers
inline const ofColor raisedHi   = hex( 0x334155 );   // hovered
inline const ofColor border     = hex( 0x475569 );
inline const ofColor text       = hex( 0xF8FAFC );   // 15.9:1 on panel
inline const ofColor textMuted  = hex( 0x94A3B8 );   // 6.3:1 on panel: labels, hints
inline const ofColor grid       = hex( 0xFFFFFF, 28 );   // floor grid, metres
inline const ofColor gridFine   = hex( 0xFFFFFF, 12 );   // half metres

// --- status: one meaning each ---
inline const ofColor ok         = hex( 0x22C55E );   // live, connected, start
inline const ofColor active     = hex( 0x3B82F6 );   // the interactive accent: selection, sliders, focus
inline const ofColor activeFill = hex( 0x1D4ED8 );   // the same, under text: a toggle that is on (white text on it, 6.7:1)
inline const ofColor warning    = hex( 0xF59E0B );   // connecting, partial (a person seen only as a box), not calibrated
inline const ofColor danger     = hex( 0xF87171 );   // recording, faults, hold: as text (5.7:1 on panel)
inline const ofColor dangerFill = hex( 0xA84848 );   // the same, as a button's fill: a soft brick red (white text on it, 5.4:1)
inline const ofColor info       = hex( 0x38BDF8 );   // playing back a recording

// --- what is drawn ---
inline const ofColor person     = hex( 0x4ADE80 );   // someone tracked
inline const ofColor fakePerson = hex( 0xA78BFA );   // the stand-in person (testing)
inline const ofColor boxPerson  = warning;           // someone found without a skeleton: partial
inline const ofColor sideLeft   = hex( 0x60A5FA );   // a body's left side
inline const ofColor sideRight  = hex( 0xF87171 );   // its right side
inline const ofColor narcissus  = hex( 0x38BDF8 );   // Narcissus and his gaze (the water)
inline const ofColor echo       = hex( 0xF472B6 );   // Echo, her gaze, and the person she is with
inline const ofColor axisX = hex( 0xEF4444 ), axisY = hex( 0x22C55E ), axisZ = hex( 0x3B82F6 );
// cameras 0..3: the reference first
inline const ofColor& camera( int i ) {
    static const ofColor c[4] = { hex( 0x22D3EE ), hex( 0xA78BFA ), hex( 0xFACC15 ), hex( 0xFB923C ) };
    return c[std::clamp( i, 0, 3 )];
}
// a joint's colour: its side's tint over the base, the spine and head left as they are (side: -1 left, 0, 1 right)
inline ofColor sided( const ofColor& aBase, int aSide ) {
    return aSide < 0 ? aBase.getLerped( sideLeft, 0.5f ) : aSide > 0 ? aBase.getLerped( sideRight, 0.5f ) : aBase;
}

// The ImGui style: slate surfaces, the blue accent, dense spacing, small rounding, no motion. Call once after the
// ImGui context exists (after gui.setup()).
inline void applyImGui() {
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowPadding = ImVec2( 10, 10 );
    s.FramePadding = ImVec2( 6, 4 );
    s.ItemSpacing = ImVec2( 8, 5 );
    s.ItemInnerSpacing = ImVec2( 6, 4 );
    s.IndentSpacing = 14;
    s.ScrollbarSize = 12;
    s.GrabMinSize = 10;
    s.WindowRounding = 4; s.ChildRounding = 4; s.FrameRounding = 4; s.PopupRounding = 4;
    s.ScrollbarRounding = 4; s.GrabRounding = 4; s.TabRounding = 4;
    s.WindowBorderSize = 1; s.FrameBorderSize = 0; s.PopupBorderSize = 1;

    ImVec4* c = s.Colors;
    c[ImGuiCol_Text]                 = im( text );
    c[ImGuiCol_TextDisabled]         = im( textMuted );
    c[ImGuiCol_WindowBg]             = im( panel, 0.97f );
    c[ImGuiCol_ChildBg]              = im( panel, 0.f );
    c[ImGuiCol_PopupBg]              = im( panel, 0.98f );
    c[ImGuiCol_Border]               = im( border, 0.6f );
    c[ImGuiCol_BorderShadow]         = ImVec4( 0, 0, 0, 0 );
    c[ImGuiCol_FrameBg]              = im( raised );
    c[ImGuiCol_FrameBgHovered]       = im( raisedHi );
    c[ImGuiCol_FrameBgActive]        = im( border );
    c[ImGuiCol_TitleBg]              = im( bg );
    c[ImGuiCol_TitleBgActive]        = im( raised );
    c[ImGuiCol_TitleBgCollapsed]     = im( bg );
    c[ImGuiCol_MenuBarBg]            = im( raised );
    c[ImGuiCol_ScrollbarBg]          = im( panel, 0.f );
    c[ImGuiCol_ScrollbarGrab]        = im( border );
    c[ImGuiCol_ScrollbarGrabHovered] = im( textMuted );
    c[ImGuiCol_ScrollbarGrabActive]  = im( text );
    c[ImGuiCol_CheckMark]            = im( active );
    c[ImGuiCol_SliderGrab]           = im( active );
    c[ImGuiCol_SliderGrabActive]     = im( active.getLerped( text, 0.3f ));
    c[ImGuiCol_Button]               = im( raisedHi );
    c[ImGuiCol_ButtonHovered]        = im( border );
    c[ImGuiCol_ButtonActive]         = im( active );
    c[ImGuiCol_Header]               = im( raised );
    c[ImGuiCol_HeaderHovered]        = im( raisedHi );
    c[ImGuiCol_HeaderActive]         = im( active, 0.6f );
    c[ImGuiCol_Separator]            = im( border, 0.6f );
    c[ImGuiCol_SeparatorHovered]     = im( active );
    c[ImGuiCol_SeparatorActive]      = im( active );
    c[ImGuiCol_ResizeGrip]           = im( border, 0.4f );
    c[ImGuiCol_ResizeGripHovered]    = im( active, 0.7f );
    c[ImGuiCol_ResizeGripActive]     = im( active );
    c[ImGuiCol_Tab]                  = im( raised );
    c[ImGuiCol_TabHovered]           = im( raisedHi );
    c[ImGuiCol_TabActive]            = im( active, 0.6f );
    c[ImGuiCol_TabUnfocused]         = im( raised );
    c[ImGuiCol_TabUnfocusedActive]   = im( raisedHi );
    c[ImGuiCol_PlotLines]            = im( active );
    c[ImGuiCol_PlotHistogram]        = im( active );
    c[ImGuiCol_TextSelectedBg]       = im( active, 0.35f );
    c[ImGuiCol_NavHighlight]         = im( active );
}

// Status text, by meaning.
inline ImVec4 statusOk()      { return im( ok ); }
inline ImVec4 statusWarning() { return im( warning ); }
inline ImVec4 statusDanger()  { return im( danger ); }
inline ImVec4 statusInfo()    { return im( info ); }

// WCAG contrast between two colours (1..21).
inline float contrast( const ofColor& a, const ofColor& b ) {
    auto lum = []( const ofColor& c ) {
        auto ch = []( float v ) { v /= 255.f; return v <= 0.03928f ? v / 12.92f : std::pow(( v + 0.055f ) / 1.055f, 2.4f ); };
        return 0.2126f * ch( c.r ) + 0.7152f * ch( c.g ) + 0.0722f * ch( c.b );
    };
    const float la = lum( a ), lb = lum( b );
    return ( std::max( la, lb ) + 0.05f ) / ( std::min( la, lb ) + 0.05f );
}
// Whichever of white and the dark background reads better on a fill.
inline ofColor textOn( const ofColor& aFill ) { return contrast( text, aFill ) >= contrast( bg, aFill ) ? text : bg; }

// A full-width primary button in a status colour (green start, deep red stop / record, ...), its text white or dark,
// whichever reads better on it. For red use dangerFill, not danger.
inline bool actionButton( const char* aLabel, const ofColor& aFill ) {
    ImGui::PushStyleColor( ImGuiCol_Button, im( aFill ));
    ImGui::PushStyleColor( ImGuiCol_ButtonHovered, im( aFill.getLerped( text, 0.15f )));
    ImGui::PushStyleColor( ImGuiCol_ButtonActive, im( aFill.getLerped( bg, 0.2f )));
    ImGui::PushStyleColor( ImGuiCol_Text, im( textOn( aFill )));
    const bool tpressed = ImGui::Button( aLabel, ImVec2( -1, 0 ));
    ImGui::PopStyleColor( 4 );
    return tpressed;
}

// An on / off setting as a push button (in place of a checkbox): on is filled with the accent, off is plain, and
// the label says which, so it does not rest on colour alone. The id stays the same whichever state it shows. A setting
// whose on is a hazard (a real arm armed) passes dangerFill for aOnFill.
inline bool toggle( const char* aLabel, bool& aValue, float aWidth = -1.f, const ofColor& aOnFill = activeFill ) {
    const std::string tlabel = std::string( aLabel ) + ( aValue ? "   on" : "   off" ) + "###" + aLabel;
    ImGui::PushStyleColor( ImGuiCol_Button, aValue ? im( aOnFill ) : im( raised ));
    ImGui::PushStyleColor( ImGuiCol_ButtonHovered, aValue ? im( aOnFill.getLerped( text, 0.12f )) : im( raisedHi ));
    ImGui::PushStyleColor( ImGuiCol_ButtonActive, aValue ? im( aOnFill.getLerped( bg, 0.2f )) : im( border ));
    ImGui::PushStyleColor( ImGuiCol_Text, aValue ? im( textOn( aOnFill )) : im( textMuted ));
    const bool tpressed = ImGui::Button( tlabel.c_str(), ImVec2( aWidth, 0 ));
    ImGui::PopStyleColor( 4 );
    if( tpressed ) aValue = !aValue;
    return tpressed;
}
inline bool toggle( ofParameter< bool >& aParam, float aWidth = -1.f ) {
    bool tv = aParam;
    if( !toggle( aParam.getName().c_str(), tv, aWidth )) return false;
    aParam = tv;
    return true;
}

}
