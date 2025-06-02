using UnityEngine;
using RosSharp.RosBridgeClient;
using TMPro;
using Instructions = RosSharp.RosBridgeClient.MessageTypes.Instructions;
using System.Collections.Generic;
using System.Collections;
using Newtonsoft.Json;

public class InstructionsService : UnityServiceProvider<Instructions.InstructionsLLMRequest, Instructions.InstructionsLLMResponse>
{
    [SerializeField] private CanvasHandler _canvas;
    private Queue<Instructions.InstructionsLLMRequest> requestsQueue = new Queue<Instructions.InstructionsLLMRequest>();

    void Start()
    {
        Debug.Log("InstructionsService: Start chamado e inicializado.");
        base.Start(); 
        if (_canvas == null)
        {
            Debug.LogWarning("InstructionsService: _canvas É NULL no Start.");
        }
        else
        {
            Debug.Log("InstructionsService: _canvas NÃO É NULL no Start.");
        }
    }

    protected override bool ServiceCallHandler(Instructions.InstructionsLLMRequest request, out Instructions.InstructionsLLMResponse response)
    {
        Debug.Log("[InstructionsService] ServiceCallHandler: INÍCIO DA CHAMADA.");
        Debug.Log("[InstructionsService] Instrução recebida: " + request.instruction);

        requestsQueue.Enqueue(request);

        response = new Instructions.InstructionsLLMResponse
        {
            success = true
        };
        Debug.Log("[InstructionsService] Requisição enfileirada. Resposta de sucesso preparada. FIM DA CHAMADA.");
        return true;
    }

    private void Update()
    {
        while (requestsQueue.Count > 0)
        {
            var request = requestsQueue.Dequeue();
            ProcessInstructionRequest(request);
        }
    }

    private void ProcessInstructionRequest(Instructions.InstructionsLLMRequest request)
    {
        Debug.Log("[InstructionsService] ProcessInstructionRequest: Processando instrução: " + request.instruction);
        if (_canvas != null)
        {
            Debug.Log("[InstructionsService] _canvas NÃO é null. Tentando chamar UpdateInstruction.");
            _canvas.UpdateInstruction(request.instruction);
            Debug.Log("[InstructionsService] UpdateInstruction chamado. Instrução enviada: " + request.instruction);
        }
        else
        {
            Debug.LogWarning("[InstructionsService] _canvas É null em ProcessInstructionRequest. Não é possível chamar UpdateInstruction.");
        }
    }

}