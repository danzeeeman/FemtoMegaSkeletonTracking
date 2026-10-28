#pragma once
#include "ofMain.h"
#include "imgui.h"

// The look of femtoSkeletonOsc and oscRecPlay: one dark, dense, quiet operator console, where a colour always means
// the same thing. The same file is in femtoSkeletonOsc/src and oscRecPlay/src.
namespace optheme {

inline ofColor hex( uint32_t aRgb, int aAlpha = 255 ) { return ofColor(( aRgb >> 16 ) & 255, ( aRgb >> 8 ) & 255, aRgb & 255, aAlpha ); }
inline ImVec4 im( const ofColor& c, float aAlpha = -1.f ) { return ImVec4( c.r / 255.f, c.g / 255.f, c.b / 255.f, aAlpha >= 0.f ? aAlpha : c.a / 255.f ); }

// --- surfaces: black, neutral greys, no tint; panels see-through over the views, parts drawn as outlines ---
inline const ofColor bg         = hex( 0x000000 );   // the window behind everything, 3D view backgrounds
inline const ofColor panel      = hex( 0x0A0A0A );   // control windows (drawn at 55%: the views show through)
inline const ofColor raised     = hex( 0xFFFFFF, 14 );   // fields, headers: a breath of white, not a solid
inline const ofColor raisedHi   = hex( 0xFFFFFF, 30 );   // hovered
inline const ofColor border     = hex( 0x6B6B6B );   // outlines of buttons, fields, panels (3.9:1 on black)
inline const ofColor text       = hex( 0xF5F5F5 );   // 19:1 on black
inline const ofColor textMuted  = hex( 0xB0B0B0 );   // 10:1 on black: labels, hints
inline const ofColor grid       = hex( 0xFFFFFF, 34 );   // floor grid, metres
inline const ofColor gridFine   = hex( 0xFFFFFF, 14 );   // half metres

// --- status: one meaning each ---
inline const ofColor ok         = hex( 0x22C55E );   // live, connected, start
inline const ofColor active     = hex( 0x7DB7FF );   // the interactive accent: a toggle that is on, focus, selection (9:1 on black)
inline const ofColor activeFill = active;            // (was a solid fill; on now draws as an outline in the accent)
inline const ofColor warning    = hex( 0xF59E0B );   // connecting, partial (a person seen only as a box), not calibrated
inline const ofColor danger     = hex( 0xF87171 );   // recording, faults, hold: as text (5.7:1 on panel)
inline const ofColor dangerFill = danger;            // (was a solid fill; buttons now draw red as an outline and text)
inline const ofColor info       = hex( 0x38BDF8 );   // playing back a recording

// --- what is drawn ---
inline const ofColor person     = hex( 0x4ADE80 );   // someone tracked
inline const ofColor fakePerson = hex( 0xA78BFA );   // the stand-in person (testing)
inline const ofColor boxPerson  = warning;           // someone found without a skeleton: partial
inline const ofColor sideLeft   = hex( 0x60A5FA );   // a body's left side
inline const ofColor sideRight  = hex( 0xF87171 );   // its right side
inline const ofColor axisX = hex( 0xEF4444 ), axisY = hex( 0x22C55E ), axisZ = hex( 0x3B82F6 );
// cameras: the reference first; eight colours, then they come round again
inline const ofColor& camera( int i ) {
    static const ofColor c[8] = { hex( 0x22D3EE ), hex( 0xA78BFA ), hex( 0xFACC15 ), hex( 0xFB923C ),
                                  hex( 0xE879F9 ), hex( 0x2DD4BF ), hex( 0xA3E635 ), hex( 0xFDA4AF ) };
    return c[std::max( i, 0 ) % 8];
}
// a joint's colour: its side's tint over the base, the spine and head left as they are (side: -1 left, 0, 1 right)
inline ofColor sided( const ofColor& aBase, int aSide ) {
    return aSide < 0 ? aBase.getLerped( sideLeft, 0.5f ) : aSide > 0 ? aBase.getLerped( sideRight, 0.5f ) : aBase;
}

// The ImGui style: black, see-through panels, outlined controls with only a trace of fill, dense spacing, small
// rounding, no motion. Call once after the ImGui context exists (after gui.setup()).
inline void applyImGui() {
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowPadding = ImVec2( 10, 10 );
    s.FramePadding = ImVec2( 7, 5 );
    s.ItemSpacing = ImVec2( 8, 6 );
    s.ItemInnerSpacing = ImVec2( 6, 4 );
    s.IndentSpacing = 14;
    s.ScrollbarSize = 12;
    s.GrabMinSize = 10;
    s.WindowRounding = 3; s.ChildRounding = 3; s.FrameRounding = 3; s.PopupRounding = 3;
    s.ScrollbarRounding = 3; s.GrabRounding = 2; s.TabRounding = 3;
    s.WindowBorderSize = 1; s.FrameBorderSize = 1; s.PopupBorderSize = 1;

    ImVec4* c = s.Colors;
    c[ImGuiCol_Text]                 = im( text );
    c[ImGuiCol_TextDisabled]         = im( textMuted );
    c[ImGuiCol_WindowBg]             = im( panel, 0.55f );
    c[ImGuiCol_ChildBg]              = ImVec4( 0, 0, 0, 0 );
    c[ImGuiCol_PopupBg]              = im( bg, 0.92f );   // a question stays readable over anything
    c[ImGuiCol_ModalWindowDimBg]     = im( bg, 0.6f );
    c[ImGuiCol_Border]               = im( border, 0.7f );
    c[ImGuiCol_BorderShadow]         = ImVec4( 0, 0, 0, 0 );
    c[ImGuiCol_FrameBg]              = im( raised );
    c[ImGuiCol_FrameBgHovered]       = im( raisedHi );
    c[ImGuiCol_FrameBgActive]        = im( active, 0.18f );
    c[ImGuiCol_TitleBg]              = im( bg, 0.7f );
    c[ImGuiCol_TitleBgActive]        = im( bg, 0.85f );
    c[ImGuiCol_TitleBgCollapsed]     = im( bg, 0.7f );
    c[ImGuiCol_MenuBarBg]            = im( raised );
    c[ImGuiCol_ScrollbarBg]          = ImVec4( 0, 0, 0, 0 );
    c[ImGuiCol_ScrollbarGrab]        = im( border, 0.8f );
    c[ImGuiCol_ScrollbarGrabHovered] = im( textMuted );
    c[ImGuiCol_ScrollbarGrabActive]  = im( text );
    c[ImGuiCol_CheckMark]            = im( active );
    c[ImGuiCol_SliderGrab]           = im( text, 0.55f );   // a soft white handle, not a block of colour
    c[ImGuiCol_SliderGrabActive]     = im( active );
    c[ImGuiCol_Button]               = im( raised );
    c[ImGuiCol_ButtonHovered]        = im( raisedHi );
    c[ImGuiCol_ButtonActive]         = im( active, 0.25f );
    c[ImGuiCol_Header]               = ImVec4( 0, 0, 0, 0 );   // section headers: the outline and the arrow only
    c[ImGuiCol_HeaderHovered]        = im( raisedHi );
    c[ImGuiCol_HeaderActive]         = im( active, 0.2f );
    c[ImGuiCol_Separator]            = im( border, 0.6f );
    c[ImGuiCol_SeparatorHovered]     = im( active );
    c[ImGuiCol_SeparatorActive]      = im( active );
    c[ImGuiCol_ResizeGrip]           = im( border, 0.4f );
    c[ImGuiCol_ResizeGripHovered]    = im( active, 0.7f );
    c[ImGuiCol_ResizeGripActive]     = im( active );
    c[ImGuiCol_Tab]                  = im( raised );
    c[ImGuiCol_TabHovered]           = im( raisedHi );
    c[ImGuiCol_TabActive]            = im( active, 0.2f );
    c[ImGuiCol_TabUnfocused]         = im( raised );
    c[ImGuiCol_TabUnfocusedActive]   = im( raisedHi );
    c[ImGuiCol_PlotLines]            = im( active );
    c[ImGuiCol_PlotHistogram]        = im( active, 0.6f );
    c[ImGuiCol_TextSelectedBg]       = im( active, 0.3f );
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

// A button outlined and lettered in a colour (green start, red stop / record, the accent ...), over a trace of the
// same colour: it reads by its words and its outline, not a block of colour.
inline void pushOutlined( const ofColor& aColour, bool aStrong = true ) {
    ImGui::PushStyleColor( ImGuiCol_Button, im( aColour, aStrong ? 0.14f : 0.06f ));
    ImGui::PushStyleColor( ImGuiCol_ButtonHovered, im( aColour, 0.26f ));
    ImGui::PushStyleColor( ImGuiCol_ButtonActive, im( aColour, 0.38f ));
    ImGui::PushStyleColor( ImGuiCol_Border, im( aColour, 0.9f ));
    ImGui::PushStyleColor( ImGuiCol_Text, im( aColour.getLerped( text, 0.25f )));
}
inline void popOutlined() { ImGui::PopStyleColor( 5 ); }

// A full-width primary button in a status colour, as an outline.
inline bool actionButton( const char* aLabel, const ofColor& aColour ) {
    pushOutlined( aColour );
    const bool tpressed = ImGui::Button( aLabel, ImVec2( -1, 0 ));
    popOutlined();
    return tpressed;
}

// An on / off setting as a push button (in place of a checkbox): on is filled with the accent, off is plain, and
// the label says which, so it does not rest on colour alone. The id stays the same whichever state it shows. A setting
// whose on is a hazard (a real arm armed) passes dangerFill for aOnFill.
inline bool toggle( const char* aLabel, bool& aValue, float aWidth = -1.f, const ofColor& aOnFill = activeFill ) {
    const std::string tlabel = std::string( aLabel ) + ( aValue ? "   on" : "   off" ) + "###" + aLabel;
    // on: outlined and lettered in the accent (or red for a hazard); off: a grey outline, muted letters
    if( aValue ) pushOutlined( aOnFill );
    else {
        ImGui::PushStyleColor( ImGuiCol_Button, ImVec4( 0, 0, 0, 0 ));
        ImGui::PushStyleColor( ImGuiCol_ButtonHovered, im( raisedHi ));
        ImGui::PushStyleColor( ImGuiCol_ButtonActive, im( aOnFill, 0.25f ));
        ImGui::PushStyleColor( ImGuiCol_Border, im( border, 0.7f ));
        ImGui::PushStyleColor( ImGuiCol_Text, im( textMuted ));
    }
    const bool tpressed = ImGui::Button( tlabel.c_str(), ImVec2( aWidth, 0 ));
    popOutlined();
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
