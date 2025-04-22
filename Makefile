# Zachary Perry
# compiles problem_1 and problem_2 into respective bin/ files
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
