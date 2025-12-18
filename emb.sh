#!/bin/bash
# emb.sh - Embassy Build & Flash Helper

# Farben 
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_info() { echo -e "${BLUE}🔧 $1${NC}"; }
print_success() { echo -e "${GREEN}✅ $1${NC}"; }
print_error() { echo -e "${RED}❌ $1${NC}"; }

CHIP="STM32G0B1RE"
PROJECT_NAME="lsm6dsv32"  # Aus Cargo.toml

# Flash-Modus (Release - läuft ohne Debugger)
flash_release() {
    local BINARY=${1:-$PROJECT_NAME}  # Default: Projektname
    
    print_info "Baue $BINARY (release)..."
    echo "================================"

    if cargo build --release --bin $BINARY; then
        print_success "Release-Build erfolgreich!"
        echo ""
        print_info "Flashe $BINARY auf $CHIP..."
        echo "================================"
        
        # Mit cargo run flashen (läuft sofort)
        if cargo run --release --bin $BINARY; then
            print_success "✅ $BINARY erfolgreich geflasht und gestartet!"
            echo ""
            print_info "Das Programm läuft JETZT auf dem Board!"
            print_info "LEDs blinken, Buttons funktionieren, etc."
        else
            print_error "Fehler beim Flashen!"
            exit 1
        fi
    else
        print_error "Build fehlgeschlagen!"
        exit 1
    fi
}

# Debug-Modus (für VSCode Debugging)
flash_debug() {
    local BINARY=${1:-$PROJECT_NAME}  # Default: Projektname
    
    print_info "Baue $BINARY (debug für VSCode)..."
    echo "================================"

    if cargo build --bin $BINARY; then
        print_success "Debug-Build erfolgreich!"
        ELF="target/thumbv6m-none-eabi/debug/$BINARY"
        echo ""
        
        print_info "Flashe $BINARY für Debugging..."
        echo "================================"
        
        if probe-rs download --chip "$CHIP" "$ELF"; then
            print_success "✅ $BINARY erfolgreich geflasht!"
            echo ""
            print_info "Jetzt in VSCode debuggen:"
            echo "  1. Öffne VSCode"
            echo "  2. Drücke F5"
            echo "  3. Wähle '$BINARY' aus der Liste"
            echo "  4. Debugger startet (Programm hält am Anfang)"
        else
            print_error "Fehler beim Flashen!"
            exit 1
        fi
    else
        print_error "Build fehlgeschlagen!"
        exit 1
    fi
}

list_binaries() {
    print_info "Verfügbare Binaries:"
    echo "===================="
    
    # Aus Cargo.toml lesen oder src/bin/ durchsuchen
    if [ -f "Cargo.toml" ]; then
        # Extrahiere bin-Namen aus Cargo.toml
        grep -A2 '\[\[bin\]\]' Cargo.toml | grep 'name = ' | cut -d'"' -f2
    fi
    
    # Falls src/bin existiert
    if [ -d "src/bin" ]; then
        find src/bin -name "*.rs" | while read file; do
            echo "  - $(basename "$file" .rs)"
        done
    fi
    
    # Hauptbinary
    echo "  - $PROJECT_NAME (main)"
}

main() {
    # Wenn kein Argument gegeben, Hilfe anzeigen
    if [ -z "$1" ] || [ "$1" = "help" ]; then
        print_info "Verwendung:"
        echo "  ./emb.sh [binary]        # Flash & Run (Release) - Default: $PROJECT_NAME"
        echo "  ./emb.sh [binary] debug  # Flash für VSCode Debugging"
        echo "  ./emb.sh list            # Zeige verfügbare Binaries"
        echo "  ./emb.sh help            # Diese Hilfe"
        echo ""
        list_binaries
        exit 0
    fi

    if [ "$1" = "list" ]; then
        list_binaries
        exit 0
    fi

    BINARY=$1
    MODE=${2:-run}  # Default: run (release)

    # Wenn nur Modus angegeben wurde (kein Binary-Name)
    if [ "$BINARY" = "debug" ] || [ "$BINARY" = "release" ] || [ "$BINARY" = "run" ]; then
        MODE=$BINARY
        BINARY=$PROJECT_NAME
    fi

    # Überprüfe ob Binary existiert
    if [ "$BINARY" != "$PROJECT_NAME" ]; then
        if [ ! -f "src/bin/${BINARY}.rs" ]; then
            print_error "Binary '$BINARY' nicht gefunden!"
            echo ""
            list_binaries
            exit 1
        fi
    fi

    case "$MODE" in
        run|release)
            flash_release "$BINARY"
            ;;
        debug)
            flash_debug "$BINARY"
            ;;
        *)
            print_error "Unbekannter Modus: $MODE (run|debug)"
            exit 1
            ;;
    esac
}

# Script ausführen
main "$@"