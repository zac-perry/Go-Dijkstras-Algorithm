# Zachary Perry
# Compiles the given code for problem 3. 
.DEFAULT_GOAL := build

fmt: 
	@go fmt ./...

lint: fmt
	@golint ./...

vet: fmt
	@go vet ./...

build: vet
	@go build -o bin/main main.go

clean:
	@go clean
	rm bin/*

.PHONY: fmt lint vet build clean
